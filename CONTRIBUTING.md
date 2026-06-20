# Contributing

Thanks for taking an interest! This is primarily a personal learning project,
but issues, suggestions and pull requests are welcome — especially if you are
building something similar and spot a mistake or a better approach.

## Ground rules

- Keep `sbr_control::BalanceController` / `Pid` **free of ROS includes**. The
  control core is shared with the ESP32-S3 micro-ROS firmware (see `CLAUDE.md`).
- Add or update a unit test in `src/sbr_control/test/` when you change the
  control law.
- Match the surrounding style: C++17 with the existing formatting, Python
  following PEP 8.

## Development setup

See [docs/setup.md](docs/setup.md). In short, on Ubuntu 24.04 with ROS 2 Jazzy:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon test --packages-select sbr_control
```

## Before opening a PR

- `colcon build --symlink-install` succeeds.
- `colcon test --packages-select sbr_control` passes.
- Python changes compile: `python3 -m py_compile <file>`.
- Update the relevant doc in `docs/` if behaviour or hardware changes.

## Commit messages

Short imperative subject (e.g. "Add D-term anti-windup guard"), with a body
explaining *why* if it isn't obvious.

## Safety

This robot has spinning motors and a Li-ion battery. When testing hardware
changes, keep the wheels clear, test gains with the robot held or on a stand
first, and make sure the tip-kill (`fall_threshold`) and low-battery cut-off
work before letting it run free.
