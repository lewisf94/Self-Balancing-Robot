//
// ESP32-S3 micro-ROS firmware for the two-wheeled self-balancing robot.
//
// This is the on-robot equivalent of sbr_control's balance_controller_node: it
// reuses the EXACT portable control core (firmware/lib/sbr_control -> the same
// sbr_control::BalanceController / Pid sources the ROS node and gtests use) and
// wraps it with hardware I/O (MPU-6050, TB6612FNG) and micro-ROS transport.
//
// Architecture (why two cores):
//   * Core 0  -- control_task: reads the IMU, runs BalanceController and drives
//                the motors at a fixed kLoopRateHz. This loop NEVER waits on the
//                network, so the robot keeps balancing even with no agent.
//   * Core 1  -- Arduino loop(): owns micro-ROS. Subscribes to /cmd_vel and
//                publishes /balance_state. If the agent drops, balancing
//                continues; the velocity command simply falls back to "hold".
//
// Safety preserved from the rest of the stack:
//   * tip-kill: BalanceController cuts effort when |pitch| > fall_threshold.
//   * cmd watchdog: stale /cmd_vel -> zero velocity (balance in place).
//   * low battery: PowerBoost LBO LOW -> motors hard-disabled (STBY LOW).
//   * IMU loss: repeated read failures -> motors stopped.
//
#include <Arduino.h>
#include <Wire.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/twist.h>
#include <sbr_msgs/msg/balance_state.h>

#include "sbr_control/balance_controller.hpp"  // reused control core
#include "sbr_config.hpp"
#include "mpu6050.hpp"
#include "tb6612.hpp"

using sbr_control::BalanceController;

// ===================== shared state (crosses the two cores) =================
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// /cmd_vel command : written by the ROS core, read by the control core.
static double g_linear_cmd = 0.0;
static double g_angular_cmd = 0.0;
static uint32_t g_last_cmd_ms = 0;

// Agent connection flag : ROS core -> control core.
static volatile bool g_agent_connected = false;

// Telemetry snapshot : control core -> ROS core.
struct TeleSnap
{
  double pitch, pitch_rate, pitch_setpoint, linear_cmd, angular_cmd, left, right;
  bool balancing;
};
static TeleSnap g_tele = {};

// ===================== control objects (owned by core 0) ====================
static BalanceController g_controller;
static Mpu6050 g_imu;
static Tb6612 g_motors;

// ===================== micro-ROS entities (owned by core 1) =================
static rcl_allocator_t g_allocator;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_subscription_t g_cmd_sub;
static geometry_msgs__msg__Twist g_cmd_msg;
static rcl_publisher_t g_state_pub;
static sbr_msgs__msg__BalanceState g_state_msg;

enum AgentState {WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED};
static AgentState g_state = WAITING_AGENT;

#define RCCHECK(fn) do {if ((fn) != RCL_RET_OK) {return false;}} while (0)
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static uint32_t _last = 0; \
    if (millis() - _last > static_cast<uint32_t>(MS)) {X; _last = millis();} \
} while (0)

// ============================= ROS core (core 1) ============================
static void cmd_vel_cb(const void * msgin)
{
  const auto * m = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  const double lx = m->linear.x;
  const double az = m->angular.z;
  const uint32_t t = millis();
  portENTER_CRITICAL(&g_mux);
  g_linear_cmd = lx;
  g_angular_cmd = az;
  g_last_cmd_ms = t;
  portEXIT_CRITICAL(&g_mux);
}

static bool create_entities()
{
  g_allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));
  RCCHECK(rclc_node_init_default(&g_node, "sbr_balance_firmware", "", &g_support));

  RCCHECK(
    rclc_publisher_init_best_effort(
      &g_state_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sbr_msgs, msg, BalanceState),
      "balance_state"));

  RCCHECK(
    rclc_subscription_init_default(
      &g_cmd_sub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));
  RCCHECK(
    rclc_executor_add_subscription(
      &g_executor, &g_cmd_sub, &g_cmd_msg, &cmd_vel_cb, ON_NEW_DATA));

  // Allocate the telemetry frame_id once.
  rosidl_runtime_c__String__assign(&g_state_msg.header.frame_id, "base_link");

  // Best-effort wall clock for telemetry stamps (no-op if the agent can't sync).
  rmw_uros_sync_session_time();
  return true;
}

static void destroy_entities()
{
  rmw_context_t * rmw_ctx = rcl_context_get_rmw_context(&g_support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&g_state_pub, &g_node);
  rcl_subscription_fini(&g_cmd_sub, &g_node);
  rclc_executor_fini(&g_executor);
  rcl_node_fini(&g_node);
  rclc_support_fini(&g_support);
}

static void publish_state()
{
  TeleSnap s;
  portENTER_CRITICAL(&g_mux);
  s = g_tele;
  portEXIT_CRITICAL(&g_mux);

  const int64_t ms = rmw_uros_epoch_millis();
  g_state_msg.header.stamp.sec = static_cast<int32_t>(ms / 1000);
  g_state_msg.header.stamp.nanosec = static_cast<uint32_t>((ms % 1000) * 1000000LL);
  g_state_msg.pitch = s.pitch;
  g_state_msg.pitch_rate = s.pitch_rate;
  g_state_msg.pitch_setpoint = s.pitch_setpoint;
  g_state_msg.linear_command = s.linear_cmd;
  g_state_msg.angular_command = s.angular_cmd;
  g_state_msg.left_effort = s.left;
  g_state_msg.right_effort = s.right;
  g_state_msg.balancing = s.balancing;
  (void)rcl_publish(&g_state_pub, &g_state_msg, NULL);
}

// ============================ control core (core 0) =========================
static void control_task(void *)
{
  // All I2C and motor I/O is confined to this core.
  g_motors.begin();
  Wire.begin(sbr_cfg::kImuSdaPin, sbr_cfg::kImuSclPin, sbr_cfg::kI2cHz);
  while (!g_imu.begin(Wire, sbr_cfg::kImuAddress, sbr_cfg::kComplementaryAlpha)) {
    g_motors.disable();                         // no IMU -> never drive
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  g_controller.set_params(sbr_cfg::make_params());

  const double dt = 1.0 / sbr_cfg::kLoopRateHz;
  const uint32_t period_ms = 1000UL / static_cast<uint32_t>(sbr_cfg::kLoopRateHz);
  const TickType_t period = pdMS_TO_TICKS(period_ms > 0 ? period_ms : 1);
  TickType_t last_wake = xTaskGetTickCount();
  int imu_fail = 0;

  for (;;) {
    vTaskDelayUntil(&last_wake, period);

    if (!g_imu.update(dt)) {
      if (++imu_fail > 5) {
        g_motors.stop();                        // lost the IMU -> stop driving
      }
      continue;
    }
    imu_fail = 0;

    double pitch = g_imu.pitch();
    double pitch_rate = g_imu.pitch_rate();
    if (sbr_cfg::kInvertPitch) {
      pitch = -pitch;
      pitch_rate = -pitch_rate;
    }

    // Snapshot the latest velocity command.
    double lin, ang;
    uint32_t last_ms;
    portENTER_CRITICAL(&g_mux);
    lin = g_linear_cmd;
    ang = g_angular_cmd;
    last_ms = g_last_cmd_ms;
    portEXIT_CRITICAL(&g_mux);

    const bool stale =
      (millis() - last_ms) > static_cast<uint32_t>(sbr_cfg::kCmdTimeoutS * 1000.0);
    if (!g_agent_connected || stale) {
      lin = 0.0;                                // balance in place
      ang = 0.0;
    }

    const bool low_batt =
      sbr_cfg::kHasLowBatteryPin && digitalRead(sbr_cfg::kPinLBO) == LOW;

    const BalanceController::Command cmd =
      g_controller.update(pitch, pitch_rate, lin, ang, dt);

    if (low_batt) {
      g_motors.disable();                       // hard cut (STBY LOW)
    } else if (!cmd.balancing) {
      g_motors.stop();                          // fallen -> coast
    } else {
      g_motors.set(cmd.left_effort, cmd.right_effort);
    }

    // Hand a snapshot to the ROS core for /balance_state.
    portENTER_CRITICAL(&g_mux);
    g_tele.pitch = pitch;
    g_tele.pitch_rate = pitch_rate;
    g_tele.pitch_setpoint = cmd.pitch_setpoint;
    g_tele.linear_cmd = lin;
    g_tele.angular_cmd = ang;
    g_tele.left = cmd.left_effort;
    g_tele.right = cmd.right_effort;
    g_tele.balancing = cmd.balancing;
    portEXIT_CRITICAL(&g_mux);
  }
}

// ================================ Arduino ==================================
void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);       // micro-ROS over USB serial

  // Start balancing on core 0; it runs whether or not the agent ever connects.
  xTaskCreatePinnedToCore(
    control_task, "control", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 0);

  g_state = WAITING_AGENT;
}

void loop()
{
  // micro-ROS connection state machine (runs on core 1; the blocking pings here
  // never stall the balance loop on core 0).
  switch (g_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(
        500,
        g_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ?
          AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        g_agent_connected = true;
        g_state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        g_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(
        500,
        g_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ?
          AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (g_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(2));
        EXECUTE_EVERY_N_MS(20, publish_state(););   // ~50 Hz telemetry
      }
      break;

    case AGENT_DISCONNECTED:
      g_agent_connected = false;
      destroy_entities();
      g_state = WAITING_AGENT;
      break;
  }
}
