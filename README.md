# ROS 2 Development with Pixi

A modern ROS 2 workspace showcasing **pixi-build-ros** - the simplest way to build and manage ROS packages.

## Quick Demo

```bash
# Clone this repository
git clone https://github.com/ruben-arts/ros_workspace.git
```

```bash
# Install and build everything
pixi install
```

```bash
# Start turtle simulator (terminal 1)
pixi run sim
```

```bash  
# Start navigation controller (terminal 2)
pixi run navigator
```

```bash
# Send movement command (terminal 3) 
pixi run talker 5.5 8.0
```

Watch the turtle smoothly navigate to your coordinates using a PID controller!

## Why pixi-build-ros?

- **Zero configuration**: No complex extra's needed, just a small pixi toml and the rest is automated.
- **Isolated builds**: Each package builds in its own clean environment
- **Cross-platform**: Same commands work on Linux, macOS, and Windows  
- **Dependency magic**: Automatically handles ROS dependencies
- **Fast iteration**: Incremental builds and smart caching

## Prerequisites

- [Pixi](https://pixi.sh/latest/) installed
- No ROS installation needed - pixi handles everything!

## This Workspace Demo

### What's Included

This repository contains a complete working example with:

- **Turtle Simulator**: Visual robot simulation environment
- **Python Talker**: Publishes target coordinates via command line
- **C++ Navigator**: PID controller that moves turtle to target
- **Python Navigator**: Alternative Python implementation

### Available Commands

```bash  
# Turtle simulator GUI
pixi run sim

# C++ navigation with PID control
pixi run navigator

# Python navigation (alternative)  
pixi run navigator-py

# Send coordinates: pixi run talker <x> <y>
pixi run talker 2.5 7.0
```

### How It Works

1. **Talker** publishes target coordinates on the `/coordinates` topic
2. **Navigator** subscribes to coordinates and turtle pose
3. **PID Controller** calculates smooth movement commands
4. **Turtle** moves smoothly to the target position

Try different coordinates and watch the intelligent navigation!

## Creating a New Workspace

### Step 1: Initialize

```bash
mkdir my_ros_workspace && cd my_ros_workspace
```

```bash
pixi init --channel https://prefix.dev/pixi-build-backends --channel https://prefix.dev/conda-forge --channel https://prefix.dev/robostack-staging
```

### Step 2: Add ROS Dependencies

```bash
pixi add ros-humble-desktop
```

### Step 3: Verify Setup

```bash
pixi run ros2 run turtlesim turtlesim_node
```

You should see a turtle appear in a graphical window!

## Adding ROS Packages

### Python Package

Create the ROS package:
```bash
pixi run ros2 pkg create --build-type ament_python --destination-directory src --node-name my_node my_python_pkg
```


```toml
[dependencies]
ros-humble-desktop = ">=0.10.0,<0.11"
ros-humble-my-python-pkg = { path = "src/my_python_pkg/package.xml" }
```

And add the `https://prefix.dev/pixi-build-backends` channel to the `channels`.

```toml
[workspace]
channels = [
    "https://prefix.dev/pixi-build-backends",
    "https://prefix.dev/conda-forge",
    "https://prefix.dev/robostack-humble",
]
```

### C++ Package

Create the ROS package:
```bash
pixi run ros2 pkg create --build-type ament_cmake --destination-directory src --node-name my_node my_cpp_pkg --dependencies rclcpp std_msgs
```


Register with workspace:
```toml
[dependencies]
ros-humble-desktop = ">=0.10.0,<0.11"
ros-humble-my-python-pkg = { path = "src/my_python_pkg/package.xml" }
ros-humble-my-cpp-pkg = { path = "src/my_cpp_pkg/package.xml" }
```

### Add Convenience Tasks

```bash
pixi task add my-python "ros2 run my_python_pkg my_node"
pixi task add my-cpp "ros2 run my_cpp_pkg my_node"
```

## Creating Distribution Packages

Build conda packages for deployment:

```bash
# Creates .conda files for distribution
pixi build
```

Share your packages:
- Upload to [prefix.dev](https://prefix.dev/channels)
- Distribute to your team
- Deploy to production environments

## Troubleshooting

### Build Issues

**Missing dependencies:**
```bash
pixi info  # Check configured channels
pixi list --explicit # List all explicitly installed packages
pixi list # List all installed packages
pixi tree -i package-name # Check dependency tree of a package
```

**Cache problems:**
```bash
pixi clean # Full clean of the current .pixi folder in the workspace
pixi clean --build # Only clear build artifacts in the workspace
pixi clean cache --build # Clear build backend cache
pixi clean cache # Only clear cache, might be deleting more then you want, so watch out!
```

### Runtime Issues

**ROS commands not found:**
```bash
pixi shell  # Enter pixi environment
ros2 --help  # Test ROS installation
```


### Getting Help

- **Pixi**: [Documentation](https://pixi.sh) | [Discord](https://discord.gg/kKV8ZxyzY4)
- **ROS 2**: [Documentation](https://docs.ros.org/) | [Answers](https://answers.ros.org/)
- **pixi-build-ros**: [GitHub](https://github.com/prefix-dev/pixi-build-backends)

---

**Happy Robot Building! 🤖✨**
