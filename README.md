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

---

## 📂 Project Structure

| File | Description |
| :--- | :--- |
| `config.yaml` | **Configuration**. Defines ROS topics, save paths, and teleop settings. Edit this first! |
| `gripper_bridge.py` | **Driver**. Connects to the Robotiq gripper via USB and exposes it as a ROS node. |
| `lerobot_recorder_pro.py` | **Main Logic**. Handles teleoperation, data synchronization, and recording. |
| `README.md` | **Documentation**. Installation and usage guide. |

---

## 🛠️ Prerequisites & Installation

### 1. System Requirements
* **OS**: Ubuntu 20.04 (Recommended)
* **ROS**: Noetic
* **Hardware**: Kuka IIWA Robot, Robotiq 2F-85 Gripper, Camera (RealSense/USB), Gamepad (Xbox/PS4).

### 2. Environment Setup (From Scratch)

We strongly recommend using Conda to manage the Python environment (Python 3.10 is required for LeRobot).

#### Step A: Create Environment
```bash
conda create -n lerobot_env python=3.10 -y
conda activate lerobot_env

#### Step B: Install Core Dependencies
# 1. Install FFmpeg
conda install -c conda-forge ffmpeg -y

# 2. Install PyTorch (Adjust CUDA version as needed, e.g., cu121)
pip3 install torch torchvision torchaudio --index-url [https://download.pytorch.org/whl/cu121](https://download.pytorch.org/whl/cu121)

# 3. Install ROS Bridge & Hardware libraries
pip install rospkg empy catkin_pkg numpy opencv-python pyyaml pymodbus==2.5.3 pyserial

#### Step C: Install LeRobot
git clone [https://github.com/huggingface/lerobot.git](https://github.com/huggingface/lerobot.git)
cd lerobot
pip install -e .
cd ..

#### Step D: Install ROS Joy Driver
sudo apt install ros-noetic-joy -y

## 🚀 Usage Guide
To start the system, you will need 4 separate terminal tabs.


