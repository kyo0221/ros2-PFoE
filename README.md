# Particle Filter on Episode (PFoE) for ROS2

Image-based Particle Filter on Episode implementation for ROS2 Humble.

This package implements a teaching and replay system using visual observations (camera images) instead of sensor readings. The system extracts features from images using a PlaceNet model and uses them for particle filter-based localization in time (episode timeline).

## Features

- **Image-based observation**: Uses camera images converted to embedding vectors via PlaceNet neural network
- **Teach and Replay**: Record demonstrations with gamepad control, then replay autonomously
- **PFoE Algorithm**: Particle filter localization in the episode timeline for robust replay
- **ROS2 Native**: Fully implemented for ROS2 Humble

## System Architecture

### Teaching Phase
1. `joy_controller.py` - Converts gamepad input to cmd_vel commands
2. `feature_extractor.py` - Processes camera images and extracts feature vectors using PlaceNet
3. `logger.py` - Records image features and actions to ROS2 bag file

### Replay Phase
1. `feature_extractor.py` - Continues to extract features from live camera feed
2. `replay` (C++) - Runs PFoE algorithm to match current observations with teaching data and outputs actions

## Prerequisites

- ROS2 Humble
- Python packages:
  - PyTorch
  - OpenCV (python3-opencv)
  - NumPy
- C++ dependencies (handled by rosdep):
  - rclcpp
  - rosbag2_cpp
  - geometry_msgs
  - sensor_msgs
  - cv_bridge

## Installation

```bash
# Create workspace
cd ~/pfoe_ws
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select Particle_Filter_on_Episode_ros2

# Source workspace
source install/setup.bash
```

## Usage

### 1. Launch the System

```bash
ros2 launch Particle_Filter_on_Episode_ros2 teach_and_replay.launch.py
```

### 2. Teaching Phase

To start recording a teaching demonstration:

```bash
# Publish teaching mode ON
ros2 topic pub --once /teaching_mode_toggle std_msgs/msg/Bool "{data: true}"
```

- Control the robot using your gamepad (button 0 as deadman switch)
- The system records image features and actions to a bag file in `~/.ros/pfoe_bags/`

To stop teaching:

```bash
# Publish teaching mode OFF
ros2 topic pub --once /teaching_mode_toggle std_msgs/msg/Bool "{data: false}"
```

### 3. Replay Phase

To start autonomous replay:

```bash
# Set the bag file path (if not using default)
ros2 param set /replay bag_file /path/to/your/bagfile

# Publish replay mode ON
ros2 topic pub --once /replay_mode std_msgs/msg/Bool "{data: true}"
```

To stop replay:

```bash
# Publish replay mode OFF
ros2 topic pub --once /replay_mode std_msgs/msg/Bool "{data: false}"
```

## Topics

### Subscribed
- `/camera/image_raw` (sensor_msgs/msg/Image) - Input camera images
- `/joy` (sensor_msgs/msg/Joy) - Gamepad input
- `/teaching_mode_toggle` (std_msgs/msg/Bool) - Toggle teaching mode
- `/replay_mode` (std_msgs/msg/Bool) - Toggle replay mode

### Published
- `/cmd_vel` (geometry_msgs/msg/Twist) - Robot velocity commands
- `/event` (Particle_Filter_on_Episode_ros2/msg/Event) - Teaching events
- `/pfoe_out` (Particle_Filter_on_Episode_ros2/msg/PfoeOutput) - PFoE algorithm output
- `/image_feature` (std_msgs/msg/Float32MultiArray) - Extracted image features
- `/teaching_mode` (std_msgs/msg/Bool) - Current teaching mode state

## Parameters

### feature_extractor
- `model_path` (string): Path to PlaceNet model file (default: package's weights/placenet.pt)
- `image_size` (int): Input image size for model (default: 85)
- `use_gpu` (bool): Use GPU if available (default: true)

### joy_controller
- `linear_scale` (double): Linear velocity scale factor (default: 0.2)
- `angular_scale` (double): Angular velocity scale factor (default: 0.098)
- `button_deadman` (int): Deadman button index (default: 0)
- `button_level_up` (int): Level up button (default: 7)
- `button_level_down` (int): Level down button (default: 6)
- `axis_linear` (int): Linear axis index (default: 1)
- `axis_angular` (int): Angular axis index (default: 0)

### logger
- `bag_directory` (string): Directory to save bag files (default: ~/.ros/pfoe_bags)

### replay
- `num_particles` (int): Number of particles for PFoE (default: 1000)
- `loop_rate` (double): Replay loop rate in Hz (default: 10.0)
- `bag_file` (string): Path to teaching bag file

## Algorithm Details

The PFoE (Particle Filter on Episode) algorithm works as follows:

1. **Initialization**: Particles are randomly distributed across the episode timeline
2. **Sensor Update**:
   - Compute likelihood of each particle based on similarity between current observation and teaching observation at that particle's position
   - Update particle weights based on likelihood
   - Normalize weights
   - Resample particles (systematic resampling)
3. **Action Selection**: Use mode method to select action from the most frequent particle position
4. **Motion Update**: Move particles forward in time with some randomness

Feature similarity is computed using cosine distance between embedding vectors:
```
distance = sqrt(2 - 2 * dot_product(feature1, feature2))
likelihood = 1 / (1 + distance)
```

## File Structure

```
Particle_Filter_on_Episode_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── weights/
│   └── placenet.pt
├── msg/
│   ├── Event.msg
│   └── PfoeOutput.msg
├── scripts/
│   ├── feature_extractor.py
│   ├── joy_controller.py
│   └── logger.py
├── src/
│   ├── Event.cpp/hpp
│   ├── Episodes.cpp/hpp
│   ├── Observation.cpp/hpp
│   ├── Particle.cpp/hpp
│   ├── ParticleFilter.cpp/hpp
│   └── replay.cpp
├── include/Particle_Filter_on_Episode_ros2/
├── launch/
│   └── teach_and_replay.launch.py
└── config/
    └── params.yaml
```

## References

Based on the original ROS1 implementation: [raspimouse_gamepad_teach_and_replay](https://github.com/rt-net/raspimouse_gamepad_teach_and_replay)

PFoE algorithm inspired by research from Ueda Lab, Chiba Institute of Technology.

## License

BSD 3-Clause License

## Author

Implemented for ROS2 Humble
