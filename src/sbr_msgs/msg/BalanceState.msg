# Telemetry from the balance controller, published every control cycle.

std_msgs/Header header

float64 pitch            # estimated tilt angle about the wheel axis [rad], 0 = upright
float64 pitch_rate       # tilt angular velocity [rad/s]
float64 pitch_setpoint   # commanded tilt target [rad]

float64 linear_command   # forward velocity request from /cmd_vel [m/s]
float64 angular_command  # yaw rate request from /cmd_vel [rad/s]

float64 left_effort      # command sent to the left wheel
float64 right_effort     # command sent to the right wheel

bool balancing           # true while actively balancing; false if fallen / disabled

# Base-link world position [m]. Sim ground truth from Gazebo odometry; 0 on
# hardware (no encoders / no position sensing).
float64 pos_x
float64 pos_y
float64 pos_z
