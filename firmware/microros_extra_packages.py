"""PlatformIO pre-build hook: expose the repo's custom ROS 2 interface packages
to the micro-ROS static-library build.

micro_ros_platformio compiles the micro-ROS client and message type support
from source the first time you build. Custom message packages (here: sbr_msgs,
which defines BalanceState) must live in the library's ``extra_packages`` folder
at that moment. This script links everything under ``firmware/extra_packages``
into that folder.

Caveat: the micro-ROS library is built and cached once. If you add/change a
custom package after a first build, force a rebuild of just the library::

    pio run -t clean_microros && pio run

(or delete ``.pio/libdeps/<env>/micro_ros_platformio``). See README.md.
"""
Import("env")  # noqa: F821  (injected by PlatformIO)

import os
import shutil

# Force C++17 for the reused control core + this firmware. Applied to CXXFLAGS
# only so the micro-ROS C sources aren't hit with a "valid for C++ but not C"
# warning. The control core itself is C++11-clean, so this is just insurance.
env.Append(CXXFLAGS=["-std=gnu++17"])  # noqa: F821


def _link_extra_packages(*_args, **_kwargs):
    project_dir = env["PROJECT_DIR"]            # noqa: F821
    libdeps_dir = env["PROJECT_LIBDEPS_DIR"]    # noqa: F821
    pioenv = env["PIOENV"]                      # noqa: F821

    src = os.path.join(project_dir, "extra_packages")
    lib = os.path.join(libdeps_dir, pioenv, "micro_ros_platformio")
    if not os.path.isdir(src):
        return
    if not os.path.isdir(lib):
        # The library hasn't been fetched yet (first pass). It will be linked on
        # the build that actually compiles the micro-ROS sources.
        print("[sbr] micro_ros_platformio not fetched yet; will link on next build")
        return

    dst = os.path.join(lib, "extra_packages")
    os.makedirs(dst, exist_ok=True)
    for name in os.listdir(src):
        real = os.path.realpath(os.path.join(src, name))
        target = os.path.join(dst, name)
        if os.path.islink(target) or os.path.isfile(target):
            os.remove(target)
        elif os.path.isdir(target):
            shutil.rmtree(target)
        try:
            os.symlink(real, target)
        except OSError:
            shutil.copytree(real, target)
        print("[sbr] micro-ROS extra package linked: {} -> {}".format(name, real))


_link_extra_packages()
