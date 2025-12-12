# 🤖 Kuka IIWA LeRobot Data Recorder

This project provides a complete, robust pipeline for collecting imitation learning datasets on a **Kuka LBR IIWA 14** robot equipped with a **Robotiq 2F-85** gripper.

It is designed to bridge **ROS Noetic** with the **[Hugging Face LeRobot](https://github.com/huggingface/lerobot)** library (v2/v3), enabling you to record teleoperated demonstrations directly into the standard format required for training policies like **ACT** or **Diffusion Policy**.

---

## ✨ Key Features

* **⚡ Native LeRobot Integration**: Saves data directly as `.parquet` (actions/states) and `.mp4` (video), automatically handling chunking and statistics.
* **🎮 Teleoperation Support**: Includes a joystick controller with a "Deadman Switch" safety mechanism for safe robot teaching.
* **🔌 Hardware Bridge**: Includes a lightweight, pure-Python driver for Robotiq grippers via Modbus RTU (no complex C++ compilation required).
* **⚙️ Configurable**: All ROS topics, paths, and robot parameters are managed via a single `config.yaml` file.
* **🛡️ Robust Error Handling**: Features auto-save on interrupt and fixes common LeRobot schema validation issues.
* **👁️ RealSense Ready**: optimized configuration for Intel RealSense cameras (RGB alignment & resolution locking).

---

## 📂 Project Structure

| File | Description |
| :--- | :--- |
| `config.yaml` | **Configuration**. Defines ROS topics, save paths, and teleop settings. Edit this first! |
| `gripper_bridge.py` | **Driver**. Connects to the Robotiq gripper via USB and exposes it as a ROS node. |
| `lerobot_recorder_pro.py` | **Main Logic**. Handles teleoperation, data synchronization, and recording. |
| `README.md` | **Documentation**. Installation and usage guide. |
| `xbox_controller.py` | **Input Module**. Handles joystick mapping, button logic, and safety switches. Isolated for easy customization. |

---

## 🛠️ Prerequisites & Installation

### 1. System Requirements
* **OS**: Ubuntu 20.04 (Recommended)
* **ROS**: Noetic
* **Hardware**: Kuka IIWA Robot, Robotiq 2F-85 Gripper, Intel RealSense Camera (D415/D435/D405), xbox handjoy.
* 
### 2. Environment Setup (From Scratch)

We strongly recommend using Conda to manage the Python environment (Python 3.10 is required for LeRobot).

#### Step A: Create Environment
```bash
conda create -n lerobot_env python=3.10 -y
conda activate lerobot_env
```
#### Step B: Install Core Dependencies
```bash
# 1. Install FFmpeg
conda install -c conda-forge ffmpeg -y

# 2. Install PyTorch (Adjust CUDA version as needed, e.g., cu121)
pip3 install torch torchvision torchaudio --index-url [https://download.pytorch.org/whl/cu121](https://download.pytorch.org/whl/cu121)

# 3. Install ROS Bridge & Hardware libraries
pip install rospkg empy catkin_pkg numpy opencv-python pyyaml pymodbus==2.5.3 pyserial

# 4. Install ROS Realsense driver
sudo apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-description

```
#### Step C: Install LeRobot
```bash
git clone [https://github.com/huggingface/lerobot.git](https://github.com/huggingface/lerobot.git)
cd lerobot
pip install -e .
cd ..
```
#### Step D: Install ROS Joy Driver
```bash
sudo apt install ros-noetic-joy -y
```
## 🚀 Usage Guide

To start the system, you will need **5 separate terminal tabs**.

### 1️⃣ Preparation
* Connect the USB gripper and Gamepad.
* Grant permission to the USB port (e.g., `/dev/ttyUSB0`):
    ```bash
    sudo chmod 777 /dev/ttyUSB0
    ```
* Edit `config.yaml` to match your ROS topics.

### 2️⃣ Start ROS & Joystick (Terminal 1)
```bash
roscore
# Open a new tab for the joy node (check if your device is js0 or js1)
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node
```

### 3️⃣ Start Robot Driver (Terminal 2)
Launch your specific Kuka driver (e.g., `iiwa_stack` or MoveIt).
```bash
# Example
rosrun iiwa_python teleop_position_iiwa01.py \
    _rate_hz:=500.0 \
    _linear_scale:=20 \
    _angular_scale:=20 \
    _deadman_button:=4 \
    _axis_vx:=1 \
    _axis_vy:=0 \
    _axis_vz:=4 \
    _axis_wz:=3 \
    _log_dir:=/home/mingxin/iiwa_logs \
    _frame_id:=iiwa_link_0
```

### 4️⃣ Start Gripper Bridge (Terminal 3)
This script communicates with the hardware.
```bash
conda activate lerobot_env
python3 gripper_bridge.py
# Look for: "✅ 夹爪激活完成！"
```
### 5️⃣ Start RealSense Camera (Terminal 4)
```bash
roslaunch realsense2_camera rs_camera.launch \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30 \
    align_depth:=false
```
### 6️⃣ Start Recorder (Terminal 5)
This script handles the recording logic.
```bash
conda activate lerobot_env
python3 lerobot_recorder_pro.py
#(it automatically imports xbox_controller.py)

```
---

## 🎮 Operation & Recording

1.  **Standby**: The script waits for all ROS topics to be active.
2.  **Teleoperation (Safety First)**:
    * The robot **will not move** unless you hold the **Deadman Switch**.
    * **Default**: Hold **LB (Left Bumper)** on the Xbox controller.
    * While holding LB, use the joysticks to move the robot.
3.  **Recording**:
    * Recording starts automatically when the script runs.
    * Perform your task (pick and place, etc.).
4.  **Stop & Save**:
    * Release the LB button to stop the robot.
    * Press **`Ctrl + C`** in Terminal 4.
    * The script will automatically finalize and save the dataset.

---

## 📊 Output Dataset

The data will be saved to the path defined in `config.yaml` (default: `./my_dataset_kuka_v1`).

**Structure:**
```text
my_dataset_kuka_v1/
├── meta/
│   ├── info.json       # Config & frame rate
│   ├── stats.json      # Normalization stats (Mean/Std)
│   └── tasks.jsonl     # Task metadata
├── episodes/           # Robot joint states & actions (.parquet)
└── videos/             # Camera footage (.mp4)
```

**Verification:**
To visualize the recorded data:
```bash
lerobot-dataset-viz --root ./my_dataset_kuka_v1 --repo-id lerobot/kuka_v1 --episode-index 0
```

---

## 🐛 Troubleshooting

* **`SerialException: Permission denied`**: Run `sudo chmod 777 /dev/ttyUSB0`.
* **`Could not load libtorchcodec`**: You are missing FFmpeg. Run `conda install -c conda-forge ffmpeg`.
* **`Missing features: {'task'}`**: Ensure you are using the provided `lerobot_recorder_pro.py`. We use a strategy where the task is passed in the data stream but removed from the feature schema to avoid validation errors in LeRobot v2.

---

*Happy Robot Learning!* 🚀
