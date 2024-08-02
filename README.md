# An example workspace for ROS development

## How to recreate a workspace like this?
Initialize the project
```
mkdir ros_workspace && cd ros_workspace
pixi init -c conda-forge -c robostack-staging
```

Add a basic set of dependencies
```
pixi add ros-humble-desktop ros-humble-turtlesim colcon-core colcon-ros "setuptools<=58.2.0" compilers ninja pkg-config cmake ros-humble-ament-cmake-auto
```

Test the basic installation, by making a task that runs turtlesim.

```
pixi run ros2 run turtlesim turtlesim_node
```
You should now see the little turtlesim gui.

## Now lets add some custom source packages
We'll add both a Python and CPP package


### Python package
```
pixi run ros2 pkg create --build-type ament_python --destination-directory src --node-name talker talker
pixi run colcon build
```

Add the ros script to the activation of the isolated environment
```
echo "[activation]
scripts = [\"install/setup.sh\"]" >> pixi.toml
```
Run the talker example code
```
pixi run ros2 run talker talker
```

### CPP package
```
pixi run ros2 pkg create --build-type ament_cmake --destination-directory src --node-name navigator navigator --dependencies rclcpp geometry_msgs turtlesim
pixi run colcon build --cmake-args -G Ninja
```

Run the navigator example code

```
pixi run ros2 run navigator navigator
```

### Simplify future runs
To simplify using pixi we can create `pixi task`s which are run in the pixi environment and are a powerful tool to create cross-platform commands.

First add them to your `pixi.toml`
```
pixi task add sim "ros2 run turtlesim turtlesim_node"
pixi task add build "colcon build --symlink-install --cmake-args -G Ninja"
pixi task add talker "ros2 run talker talker"
pixi task add navigator "ros2 run navigator navigator"
```



## Let's add some logic
We now want our packages to have some actual code.
```py
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import argparse

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher_ = self.create_publisher(Point, 'coordinates', 10)

    def publish_coordinates(self, x, y):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0  # Assuming z is not used, set to 0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={x}, y={y}')

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()

    parser = argparse.ArgumentParser(description='Send coordinates over a ROS2 topic')
    parser.add_argument('x', type=float, help='X coordinate')
    parser.add_argument('y', type=float, help='Y coordinate')


    args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])


    try:
        node.publish_coordinates(args.x, args.y)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## CPP visualizer
```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class TurtleNavigator : public rclcpp::Node
{
public:
    TurtleNavigator()
        : Node("turtle_navigator"), x_goal_(5.0), y_goal_(5.0), kp_(1.0), ki_(0.0), kd_(0.05), prev_error_(0.0), integral_(0.0)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "coordinates", 10, std::bind(&TurtleNavigator::goal_callback, this, std::placeholders::_1));
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleNavigator::pose_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TurtleNavigator::control_loop, this));
    }

private:
    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        x_goal_ = msg->x;
        y_goal_ = msg->y;
        RCLCPP_INFO(this->get_logger(), "Received goal: x=%f, y=%f", x_goal_, y_goal_);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x_current_ = msg->x;
        y_current_ = msg->y;
        theta_current_ = msg->theta;
    }

    void control_loop()
    {
        double error_x = x_goal_ - x_current_;
        double error_y = y_goal_ - y_current_;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

        double angle_to_goal = std::atan2(error_y, error_x);
        double angle_error = angle_to_goal - theta_current_;
        
        // Normalize angle error to the range [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // PID control
        double control_signal = kp_ * distance_error + ki_ * integral_ + kd_ * (distance_error - prev_error_);
        integral_ += distance_error;
        prev_error_ = distance_error;

        // Limit control signal
        double max_linear_speed = 2.0; // Max linear speed
        double max_angular_speed = 2.0; // Max angular speed
        control_signal = std::clamp(control_signal, -max_linear_speed, max_linear_speed);

        // Publish velocity commands
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = control_signal;
        msg.angular.z = 4.0 * angle_error; // simple P controller for angle
        msg.angular.z = std::clamp(msg.angular.z, -max_angular_speed, max_angular_speed);

        publisher_->publish(msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_goal_, y_goal_;
    double x_current_, y_current_, theta_current_;
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleNavigator>());
    rclcpp::shutdown();
    return 0;
}
```

Now you can build and run these tools are "real" robotics tools.

- Build workspace
    ```shell
    pixi run build
    ```

- Start simulator
    ```shell
    pixi run sim
    ```
- Start navigation server
    ```shell
    pixi run navigator
    ```

- Give a coordinate command
    ```shell
    pixi run talker 2 3
    ```

## Now lets build these packages into a new `conda` package for later re-use in deployment.

To add the build tool (`rattler-build`) run the following:
```
pixi add rattler-build -f build
pixi project environment add build -f build --no-default-feature
```

You'll need to add a recipe to both packages in `ros_workspace/src/navigator/recipe.yaml` and `ros_workspace/src/talker/recipe.yaml`

<details>
  <summary>Recipes: (uncolapse)</summary>

  ### `ros_workspace/src/navigator/recipe.yaml`:
  ```yaml
  package:
    name: navigator
    version: 0.0.1

  source:
    path: .

  build:
    number: 0
    script: ros-conda-build.sh

  requirements:
    build:
      - ros-conda-build-support
      - ${{ compiler('cxx') }} 
      - cmake
      - ninja
      - make
      - coreutils
    host: 
      - cmake
      - ninja
      - make
      - coreutils
      - pkg-config
      - ros-humble-ament-cmake
      - ros-humble-ament-index-cpp
      - ros-humble-rclcpp
      - ros-humble-rclcpp-action
      - ros-humble-ros-environment
      - ros-humble-ros-workspace
      - ros-humble-rosidl-default-generators
      - ros-humble-std-msgs
      - ros-humble-geometry-msgs
      - ros-humble-turtlesim
      - ros2-distro-mutex 0.5 humble
    run :
      - ros-humble-rclcpp
      - ros-humble-ros-workspace
      - ros-humble-rosidl-default-runtime
      - ros-humble-geometry-msgs
      - ros-humble-turtlesim
      - ros2-distro-mutex 0.5 humble

  about:
    description: Example ros package which includes a navigation server
  ```

  ### `ros_workspace/src/talker/recipe.yaml`:
  ```yaml
  package:
    name: talker
    version: 0.0.1

  source:
    path: .

  build:
    number: 0
    script: py-ros-conda-build.sh


  requirements:
    build:
      - ros-conda-build-support
      - python 3.11.*
      - setuptools <=58.2.0
      - ${{ compiler('cxx') }} 
      - cmake
      - ninja
      - colcon-core
      - colcon-ros
      - make
      - coreutils
    host: 
      - python 3.11.*
      - setuptools <=58.2.0
      - colcon-core
      - colcon-ros
      - ros-humble-ament-package
      - ros-humble-rclpy
      - ros-humble-ros-environment
      - ros-humble-ros-workspace
      - ros-humble-rosidl-default-generators
      - ros2-distro-mutex 0.5 humble
    run :
      - python 3.11.*
      - ros-humble-rclpy
      - ros-humble-ament-package
      - ros-humble-ros-workspace
      - ros-humble-rosidl-default-runtime
      - ros2-distro-mutex 0.5 humble

  about:
    description: Example ros package which includes a talker cli app
  ```
</details>


#### Add the build tasks:
```shell
pixi task add -f build build-talker "rattler-build build -r src/talker/recipe.yaml -c conda-forge -c robostack-staging -c https://prefix.dev/ros-workspace-test"
pixi task add -f build build-navigator "rattler-build build -r src/navigator/recipe.yaml -c conda-forge -c robostack-staging -c https://prefix.dev/ros-workspace-test"
```

### Run the build
```shell
pixi run build-talker
pixi run build-navigator
```

Now you have two `conda` packages build:
```shell
❯ tree output/linux-64
output/linux-64
├── navigator-0.0.1-hb0f4dca_0.conda
├── repodata.json
└── talker-0.0.1-hb0f4dca_0.conda

1 directory, 3 files
```

These can be used by adding `./output` to the `channels = []` in the pixi manifest (`pixi.toml`).

Or you can upload them to your own channel on https://prefix.dev/channels

If you uploaded them you can create another production project with the following `pixi.toml`

```toml
[project]
name = "ros_workspace_production"
version = "0.1.0"
# Example with a prebuild example, but you can switch your personal channel or local folder
# - "./PATH/TO/ABOVE_EXAMPLE/output"
# - "https://prefix.dev/your-channel"
channels = ["conda-forge", "robostack-staging", "https://prefix.dev/ros-workspace-test"]
platforms = ["linux-64"]

[tasks]
sim = "ros2 run turtlesim turtlesim_node"
navigator = "ros2 run navigator navigator"
talker = {cmd = "ros2 run talker talker", description = "Add two numbers, separated by a space"}

[dependencies]
ros-humble-desktop = ">=0.10.0,<0.11"
ros-humble-turtlesim = ">=1.4.2,<2"

navigator = "*"
talker = "*"
```

And run the example with:
```shell
pixi run sim
# other terminal
pixi run navigator
# other terminal
pixi run talker
```
