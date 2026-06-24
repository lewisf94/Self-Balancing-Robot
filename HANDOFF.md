# Handoff — continuing in Claude Code on the laptop

A short guide for picking this project up in a **fresh Claude Code session**
(for example the VS Code extension on the Ubuntu dev laptop). A new session has
the full repo but **none of the prior chat history**, so this file carries the
context across the gap.

## Where things stand

- All code and docs are on `main` and pushed — a clean `git clone` gets
  everything.
- Current focus: **get the Gazebo simulation running** on Ubuntu 24.04 (Noble)
  with ROS 2 Jazzy. No hardware involved yet.
- On a fresh laptop, **ROS 2 is probably not installed** — that's the first job.
- The control core, ESP32-S3 firmware, CI, and docs are already in place. See the
  repo map in [`GETTING_STARTED.md`](GETTING_STARTED.md).

## First message to a fresh session

Open Claude Code inside this folder (so it reads `CLAUDE.md` automatically), then
paste this to give it its bearings:

> I'm a beginner setting up this self-balancing robot project for the first time
> on Ubuntu 24.04 (Noble). I want to get the **Gazebo simulation** running — no
> hardware yet. ROS 2 Jazzy is **not installed yet**. Walk me through it step by
> step, and when commands fail, help me read the errors. Start by checking what's
> already installed, then guide me through installing ROS 2 Jazzy, building the
> workspace, and running `make sim`.

## The path (what "set up and running" means)

1. **Install ROS 2 Jazzy** on Ubuntu 24.04. Official guide:
   <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html> — choose
   `ros-jazzy-desktop`. (Let the live session run and adapt these commands; the
   install procedure is version-sensitive, so trust the official page over any
   pasted snippet.)
2. **Install project extras + build** — the simulation needs the Gazebo and
   `ros2_control` packages, then:
   ```bash
   source /opt/ros/jazzy/setup.bash
   make deps
   make build
   ```
3. **Run the sim** — `make sim`. Drive it from a second, sourced terminal with
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

Full copy-paste walkthrough with the exact package list:
[`GETTING_STARTED.md` → "Run the simulation"](GETTING_STARTED.md#run-the-simulation).

## What success looks like

- A Gazebo window with a **blue box on two black wheels**, standing upright (a
  little jitter is the controller working).
- `ros2 topic echo /balance_state` shows `balancing: true` with a small `pitch`.
- Teleop keys (`i`, `,`, `j`, `l`) lean and drive it.

If it misbehaves, grab the output of `make sim` plus `ros2 topic list` and
`ros2 topic echo /imu/data --once` — that pinpoints most problems. The
troubleshooting table in
[`docs/simulation.md`](docs/simulation.md#troubleshooting) covers the usual
suspects.

## Ground rules for the session (also in `CLAUDE.md`)

- **Commit straight to `main` and push after every change.** No new branches or
  PRs unless explicitly asked.
- **Commit as** `lewisf94 <85638536+lewisf94@users.noreply.github.com>` — never
  put a personal email in commits or files.
- **Keep the control core ROS-free** (`sbr_control::BalanceController` / `Pid`) —
  it's shared verbatim with the ESP32-S3 firmware by symlink.

## Key docs

| File | What |
|------|------|
| [`GETTING_STARTED.md`](GETTING_STARTED.md) | Beginner entry point + sim runbook |
| [`docs/setup.md`](docs/setup.md) | Full install / build / run |
| [`docs/simulation.md`](docs/simulation.md) | Sim detail + troubleshooting |
| [`docs/control_tuning.md`](docs/control_tuning.md) | Tuning the balance |
| [`docs/architecture.md`](docs/architecture.md) | How the pieces fit together |
| [`docs/hardware.md`](docs/hardware.md) | BOM, wiring, GPIO map (for later) |
| [`CLAUDE.md`](CLAUDE.md) | Conventions for any agent working here |
