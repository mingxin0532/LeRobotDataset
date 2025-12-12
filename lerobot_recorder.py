#!/usr/bin/env python3
import time
import numpy as np
import torch
import shutil
import os
import threading
from pathlib import Path
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from iiwa_msgs.msg import JointPosition
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ================= 配置 =================
TOPIC_ARM = "/iiwa/state/JointPosition"
TOPIC_GRIPPER = "/robotiq/position"
TOPIC_CAMERA = "/camera/color/image_raw"
DATASET_ROOT = "./my_dataset_final_success" # 换个吉利的名字
REPO_ID = "lerobot/kuka_final_success"
STATE_DIM = 8
CAMERA_SHAPE = (3, 480, 640)
FPS = 30
# =======================================

class RosRecorder:
    def __init__(self):
        rospy.init_node('lerobot_custom_recorder', anonymous=True)
        self.lock = threading.Lock()
        self.latest_arm_pos = None
        self.latest_gripper_pos = 0.0
        self.latest_image = None

        rospy.Subscriber(TOPIC_ARM, JointPosition, self.arm_cb)
        rospy.Subscriber(TOPIC_GRIPPER, Float32, self.gripper_cb)
        rospy.Subscriber(TOPIC_CAMERA, Image, self.camera_cb)

        rospy.loginfo("Waiting for ROS topics...")
        while (self.latest_arm_pos is None or self.latest_image is None):
            time.sleep(0.1)
            if rospy.is_shutdown(): return
        rospy.loginfo(">>> ROS Connected!")

    def arm_cb(self, msg):
        try:
            p = msg.position
            joints = [p.a1, p.a2, p.a3, p.a4, p.a5, p.a6, p.a7]
            with self.lock:
                self.latest_arm_pos = np.array(joints, dtype=np.float32)
        except: pass

    def gripper_cb(self, msg):
        with self.lock:
            self.latest_gripper_pos = float(msg.data)

    def camera_cb(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            with self.lock:
                self.latest_image = img
        except: pass

    def get_frame_data(self):
        with self.lock:
            if self.latest_arm_pos is None or self.latest_image is None: return None

            state_vec = np.concatenate([self.latest_arm_pos, [self.latest_gripper_pos]]).astype(np.float32)
            img_tensor = torch.from_numpy(self.latest_image.copy()).permute(2, 0, 1)
            action_vec = torch.from_numpy(state_vec)
            state_tensor = torch.from_numpy(state_vec)

        return {
            "observation.state": state_tensor,
            "observation.images.camera_primary": img_tensor,
            "action": action_vec,
            "task": "control robot" 
        }

def main():
    recorder = RosRecorder()

    features = {
        "observation.state": {"dtype": "float32", "shape": (STATE_DIM,), "names": ["joint"]},
        "observation.images.camera_primary": {"dtype": "video", "shape": CAMERA_SHAPE, "names": ["c","h","w"]},
        "action": {"dtype": "float32", "shape": (STATE_DIM,), "names": ["action"]},
    }

    if Path(DATASET_ROOT).exists(): shutil.rmtree(DATASET_ROOT)

    dataset = LeRobotDataset.create(
        repo_id=REPO_ID,
        fps=FPS,
        root=DATASET_ROOT,
        features=features,
        use_videos=True
    )

    rospy.loginfo(">>> Recording Started! (Ctrl+C to stop)")
    rate = rospy.Rate(FPS)
    frames = 0

    try:
        while not rospy.is_shutdown():
            data = recorder.get_frame_data()
            if data:
                dataset.add_frame(data)
                frames += 1
                if frames % 30 == 0:
                    print(f"🎥 Recorded {frames} frames...", end="\r")
            rate.sleep()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        rospy.logerr(f"CRASH: {e}")
    finally:
        rospy.loginfo("\nStopping... Saving dataset...")
        
        # 🟢 【核心修正】 使用本地计数器 frames 判断，而不是 dataset.num_frames
        if frames > 0:
            dataset.save_episode()
            rospy.loginfo(f"✅ SUCCESS! Saved {frames} frames to: {Path(DATASET_ROOT).absolute()}")
        else:
            rospy.logerr(f"FAILED: No frames recorded (frames={frames}).")

if __name__ == "__main__":
    main()
