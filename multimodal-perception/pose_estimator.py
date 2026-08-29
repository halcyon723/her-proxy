"""人体姿态估计：用 MediaPipe Pose 从图像中提取 33 个关键点。

输入：BGR 图像（OpenCV 默认格式）
输出：MediaPipe 的 pose_landmarks 对象（33 个关键点，每个含 x/y/z/visibility）
"""

import cv2
import mediapipe as mp

# 33 个关键点的名称（顺序与 MediaPipe 索引一致）
LANDMARK_NAMES = [
    "nose", "left_eye_inner", "left_eye", "left_eye_outer",
    "right_eye_inner", "right_eye", "right_eye_outer",
    "left_ear", "right_ear", "mouth_left", "mouth_right",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_pinky", "right_pinky",
    "left_index", "right_index", "left_thumb", "right_thumb",
    "left_hip", "right_hip", "left_knee", "right_knee",
    "left_ankle", "right_ankle", "left_heel", "right_heel",
    "left_foot_index", "right_foot_index",
]


class PoseEstimator:
    def __init__(self, static_image_mode=False, model_complexity=1,
                 min_detection_confidence=0.5, min_tracking_confidence=0.5):
        """初始化姿态估计器。

        static_image_mode: True=单张图片，False=视频流（带跟踪，更快）
        model_complexity: 0/1/2，越大越准越慢，1 是速度与精度的平衡点
        """
        self._pose = mp.solutions.pose.Pose(
            static_image_mode=static_image_mode,
            model_complexity=model_complexity,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def detect(self, frame_bgr):
        """输入一帧 BGR 图像，返回 pose_landmarks；未检测到人则返回 None。"""
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self._pose.process(rgb)
        return results.pose_landmarks

    def close(self):
        self._pose.close()
