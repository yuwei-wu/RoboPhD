# ROS 1 vs ROS 2 Command Comparison

This page compares commonly used commands between ROS 1 and ROS 2 across different categories.

---

### 🧭 Basic Setup

| Task                     | ROS 1                              | ROS 2                             |
|--------------------------|-------------------------------------|------------------------------------|
| Source environment       | `source devel/setup.bash`          | `source install/setup.bash`       |
| Start core/master        | `roscore`                          | _Not needed (uses DDS)_           |
| Build workspace          | `catkin_make` or `catkin build`    | `colcon build`                    |

---

### 📦 Packages & Nodes

| Task                     | ROS 1                              | ROS 2                             |
|--------------------------|-------------------------------------|------------------------------------|
| List packages            | `rospack list`                     | `ros2 pkg list`                   |
| Find package path        | `rospack find <pkg>`               | `ros2 pkg prefix <pkg>`           |
| List running nodes       | `rosnode list`                     | `ros2 node list`                  |
| Get node info            | `rosnode info <node>`              | `ros2 node info <node>`           |

---

### 📡 Topics

| Task                     | ROS 1                              | ROS 2                             |
|--------------------------|-------------------------------------|------------------------------------|
| List topics              | `rostopic list`                    | `ros2 topic list`                 |
| Echo topic               | `rostopic echo <topic>`            | `ros2 topic echo <topic>`         |
| Get topic info           | `rostopic info <topic>`            | `ros2 topic info <topic>`         |
| Publish to topic         | `rostopic pub <topic> <type>`      | `ros2 topic pub <topic> <type>`   |

---

### 📨 Services

| Task                     | ROS 1                              | ROS 2                             |
|--------------------------|-------------------------------------|------------------------------------|
| List services            | `rosservice list`                  | `ros2 service list`               |
| Service info             | `rosservice info <srv>`            | `ros2 service info <srv>`         |

