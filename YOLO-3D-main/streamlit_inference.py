# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

import io
import os
import sys
import math  # [新增] 数学库
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Tuple
from collections import defaultdict, deque

import cv2
import numpy as np
import torch

# 确保使用本地修改的ultralytics版本
current_dir = os.path.dirname(os.path.abspath(__file__))
ultralytics_root = os.path.dirname(os.path.dirname(current_dir))
if ultralytics_root not in sys.path:
    sys.path.insert(0, ultralytics_root)

from ultralytics import YOLO
from ultralytics.utils import LOGGER
from ultralytics.utils.checks import check_requirements

torch.classes.__path__ = []  # Torch module __path__._path issue fix


class Inference:
    """
    Streamlit Interface for YOLO object detection.
    Now includes Monocular Distance Estimation (Triangulation Logic).
    """

    def __init__(self, **kwargs: Any) -> None:
        """
        Initialize the Inference class.
        """
        check_requirements("streamlit>=1.29.0")
        import streamlit as st

        self.st = st
        self.source = None
        self.img_file_names = []
        self.enable_trk = False
        self.conf = 0.25
        self.iou = 0.45
        self.org_frame = None
        self.ann_frame = None
        self.vid_file_name = None
        self.selected_ind: List[int] = []
        self.model = None

        # Multi-camera support attributes
        self.num_cameras = 4
        self.camera_indices = [0, 2, 4, 6]
        
        # [新增] 相机安装朝向偏移 (机体坐标系)
        self.camera_yaw_offsets = {
            0: 180.0,  # 后
            2: -90.0,  # 右
            4: 0.0,    # 前
            6: 90.0    # 左
        }

        self.frame_width = 640
        self.frame_height = 480  # 注意：内参通常对应特定分辨率，如果分辨率改变，内参需缩放
        self.camera_caps: List[cv2.VideoCapture] = []
        self.camera_threads: List[threading.Thread] = []
        self.camera_frames: Dict[int, np.ndarray] = {}
        self.camera_running = False
        self.camera_containers: Dict[int, Tuple[Any, Any]] = {}

        # [新增] 空间测量参数 (Triangulation / Monocular Estimation)
        # 相机内参 (基于 640x480 的标定值，如果在 640x360 下使用可能需要微调 cy)
        self.camera_intrinsics = {
            'fx': 498.0,
            'fy': 498.0,
            'cx': 331.2797,
            'cy': 156.1371
        }

        # [新增] 不同类别的真实宽度 (米)
        # 必须根据实际物体尺寸修改此处，否则测距不准
        self.class_real_widths = {
            6: 0.23,  # 假设 Class 6 是特定物体，宽度 23cm
            **{i: 0.31 for i in range(6)}  # 默认 0-5 类宽度为 31cm
            # ... 添加其他类别
        }
        self.default_real_width = 0.31  # 兜底默认宽度

        # FPS monitoring attributes
        self.camera_fps: Dict[int, float] = {}
        self.camera_frame_times: Dict[int, List[float]] = {}
        self.fps_containers: Dict[int, Any] = {}
        self.overall_fps_container = None

        # Detection result saving attributes
        self.save_detections = False
        self.save_images = True
        self.save_labels = True
        self.save_directory = "./detection_results"
        self.detection_count: Dict[int, int] = {}

        # Detection Statistics
        self.class_detection_stats: Dict[int, Dict[str, int]] = {}
        self.last_detection_time: Dict[int, float] = {}

        # Display options
        self.show_class_names = True

        self.temp_dict = {"model": None, **kwargs}
        self.model_path = None
        if self.temp_dict["model"] is not None:
            self.model_path = self.temp_dict["model"]

        LOGGER.info(f"Ultralytics Solutions: ✅ {self.temp_dict}")

    # [新增] 核心测距与测角函数
    def calculate_spatial_info(self, bbox_xyxy, class_id, cam_index=0):
        """
        计算目标的距离和方位角
        :param bbox_xyxy: [x1, y1, x2, y2]
        :param class_id: 类别ID
        :param cam_index: 摄像头索引 (用于计算全局角度)
        :return: distance (m), azimuth (deg, 相对相机), global_angle (deg, 机体系)
        """
        x1, y1, x2, y2 = bbox_xyxy

        # 1. 计算像素参数
        box_width_px = x2 - x1
        x_center_px = (x1 + x2) / 2

        if box_width_px <= 0:
            return float('inf'), 0.0, 0.0

        # 2. 获取真实物理宽度
        real_width = self.class_real_widths.get(class_id, self.default_real_width)

        # 3. 距离计算 (相似三角形: Z = (W_real * f) / W_pixel)
        # 注意：如果当前分辨率不是标定分辨率，fx 应该按比例缩放
        # 这里假设 fx 是针对当前宽度的
        fx = self.camera_intrinsics['fx']
        distance = (real_width * fx) / box_width_px

        # 4. 方位角计算 (Atan模型)
        cx = self.camera_intrinsics['cx']
        pixel_offset = x_center_px - cx
        angle_rad = math.atan(pixel_offset / fx)
        azimuth_deg = math.degrees(angle_rad)

        # 5. 全局角度 (加上相机安装角度)
        cam_offset = self.camera_yaw_offsets.get(cam_index, 0.0)
        global_angle = (azimuth_deg + cam_offset + 360) % 360

        return distance, azimuth_deg, global_angle

    def web_ui(self) -> None:
        """Set up the Streamlit web interface."""
        menu_style_cfg = """<style>MainMenu {visibility: hidden;}</style>"""

        main_title_cfg = """<div><h1 style="color:#111F68; text-align:center; font-size:40px; margin-top:-50px;
        font-family: 'Archivo', sans-serif; margin-bottom:20px;">Multi-Camera Spatial Detection</h1></div>"""

        sub_title_cfg = """<div><h5 style="color:#042AFF; text-align:center; font-family: 'Archivo', sans-serif;
        margin-top:-15px; margin-bottom:50px;">Real-time detection & Distance Estimation 🚗📹📏</h5></div>"""

        self.st.set_page_config(page_title="Ultralytics Streamlit App", layout="wide")
        self.st.markdown(menu_style_cfg, unsafe_allow_html=True)
        self.st.markdown(main_title_cfg, unsafe_allow_html=True)
        self.st.markdown(sub_title_cfg, unsafe_allow_html=True)

    def sidebar(self) -> None:
        """Configure the Streamlit sidebar settings."""
        with self.st.sidebar:
            logo = "https://raw.githubusercontent.com/ultralytics/assets/main/logo/Ultralytics_Logotype_Original.svg"
            self.st.image(logo, width=250)

        self.st.sidebar.title("User Configuration")

        self.source = self.st.sidebar.selectbox(
            "Source",
            ("multi-webcam", "webcam", "video", "image"),
            index=0
        )

        # Camera configuration
        if self.source in ["webcam", "multi-webcam"]:
            self.st.sidebar.subheader("Camera Settings")

            if self.source == "multi-webcam":
                self.st.sidebar.info("💡 **Multi-Camera Setup**: Optimized for 4 cameras.")
                self.num_cameras = self.st.sidebar.slider("Number of Cameras", 1, 8, 4)

                if self.num_cameras <= 4:
                    available_cameras = [0, 2, 4, 6]
                    self.camera_indices = available_cameras[:self.num_cameras]
                else:
                    preset_configs = {
                        "Sequential (0,1,2,3,4,5,6,7)": ",".join(map(str, range(self.num_cameras))),
                        "Reliable (0,2,4,6)": "0,2,4,6",
                        "Custom": "custom"
                    }
                    selected_preset = self.st.sidebar.selectbox("Camera Configuration", list(preset_configs.keys()))

                    if selected_preset == "Custom":
                        camera_indices_str = self.st.sidebar.text_input(
                            "Camera Indices (comma-separated)",
                            ",".join(map(str, self.camera_indices[:self.num_cameras]))
                        )
                    else:
                        camera_indices_str = preset_configs[selected_preset]

                    try:
                        self.camera_indices = [int(x.strip()) for x in camera_indices_str.split(",")][:self.num_cameras]
                        self.num_cameras = len(self.camera_indices)
                    except ValueError:
                        self.st.sidebar.error("Invalid camera indices format. Using default values.")
                        self.camera_indices = [0, 2, 4, 6]
                        self.num_cameras = 4

            # Resolution settings
            if self.source == "multi-webcam":
                self.frame_width = 640
                self.frame_height = 480
                self.st.sidebar.info(f"🎯 **Resolution**: {self.frame_width}x{self.frame_height}")
            else:
                resolution_presets = {
                    "640x360": (640, 360),
                    "640x480": (640, 480),
                    "1280x720": (1280, 720),
                    "1920x1080": (1920, 1080),
                    "Custom": None
                }
                current_resolution = f"{self.frame_width}x{self.frame_height}"
                if current_resolution not in resolution_presets:
                    current_resolution = "Custom"

                selected_resolution = self.st.sidebar.selectbox(
                    "Resolution",
                    list(resolution_presets.keys()),
                    index=0
                )

                if selected_resolution == "Custom":
                    new_width = self.st.sidebar.number_input("Width", min_value=320, max_value=3840, value=self.frame_width)
                    new_height = self.st.sidebar.number_input("Height", min_value=240, max_value=2160, value=self.frame_height)
                    self.frame_width = int(new_width)
                    self.frame_height = int(new_height)
                else:
                    self.frame_width, self.frame_height = resolution_presets[selected_resolution]

        if self.source in ["webcam", "multi-webcam", "video"]:
            self.enable_trk = self.st.sidebar.radio("Enable Tracking", ("Yes", "No")) == "Yes"

        self.conf = float(self.st.sidebar.slider("Confidence Threshold", 0.0, 1.0, self.conf, 0.01))
        self.iou = float(self.st.sidebar.slider("IoU Threshold", 0.0, 1.0, self.iou, 0.01))

        # Display options
        self.st.sidebar.subheader("Display Options")
        self.show_class_names = self.st.sidebar.checkbox(
            "Show Class Names",
            value=self.show_class_names,
            help="If unchecked, only class IDs will be shown instead of class names"
        )
        # [新增] 显示距离开关
        self.show_distance = self.st.sidebar.checkbox(
            "Show Distance & Angle",
            value=True,
            help="Show calculated distance (m) and angle (deg) on the image"
        )

        # Detection saving configuration
        self.st.sidebar.subheader("Detection Saving")
        self.save_detections = self.st.sidebar.checkbox("Save Detection Results", value=self.save_detections)

        if self.save_detections:
            self.save_directory = self.st.sidebar.text_input("Save Directory", value=self.save_directory)
            self.save_images = self.st.sidebar.checkbox("Save Images", value=self.save_images)
            self.save_labels = self.st.sidebar.checkbox("Save Labels (YOLO format)", value=self.save_labels)

            if self.save_directory:
                Path(self.save_directory).mkdir(parents=True, exist_ok=True)
                self.st.sidebar.success(f"Save directory: {self.save_directory}")

        # Create UI layout based on source type
        if self.source == "multi-webcam":
            self.overall_fps_container = self.st.empty()
            cols_per_row = 2 if self.num_cameras <= 4 else 3
            rows = (self.num_cameras + cols_per_row - 1) // cols_per_row

            for row in range(rows):
                cols = self.st.columns(cols_per_row)
                for col_idx in range(cols_per_row):
                    cam_idx = row * cols_per_row + col_idx
                    if cam_idx < self.num_cameras:
                        with cols[col_idx]:
                            self.st.markdown(f"**Camera {self.camera_indices[cam_idx]}**")
                            fps_container = self.st.empty()
                            org_container = self.st.empty()
                            ann_container = self.st.empty()
                            self.camera_containers[cam_idx] = (org_container, ann_container)
                            self.fps_containers[cam_idx] = fps_container
        elif self.source != "image":
            col1, col2 = self.st.columns(2)
            self.org_frame = col1.empty()
            self.ann_frame = col2.empty()

    def source_upload(self) -> None:
        """Handle video file uploads."""
        from ultralytics.data.utils import IMG_FORMATS, VID_FORMATS

        self.vid_file_name = ""
        if self.source == "video":
            vid_file = self.st.sidebar.file_uploader("Upload Video File", type=VID_FORMATS)
            if vid_file is not None:
                g = io.BytesIO(vid_file.read())
                with open("ultralytics.mp4", "wb") as out:
                    out.write(g.read())
                self.vid_file_name = "ultralytics.mp4"
        elif self.source == "webcam":
            self.vid_file_name = 0
        elif self.source == "image":
            import tempfile
            imgfiles = self.st.sidebar.file_uploader("Upload Image Files", type=IMG_FORMATS, accept_multiple_files=True)
            if imgfiles:
                for imgfile in imgfiles:
                    with tempfile.NamedTemporaryFile(delete=False, suffix=f".{imgfile.name.split('.')[-1]}") as tf:
                        tf.write(imgfile.read())
                        self.img_file_names.append({"path": tf.name, "name": imgfile.name})

    def configure(self) -> None:
        """Configure the model and load selected classes."""
        available_models = ["Custom Car Detection Model (best.engine)"]
        custom_model_path = "/home/nvidia/Downloads/Ros/ballCar2/weights/weights/best.engine"

        if self.model_path:
            available_models.insert(0, self.model_path)

        selected_model = self.st.sidebar.selectbox("Model", available_models, index=0)

        with self.st.spinner("Model is loading..."):
            if selected_model == "Custom Car Detection Model (best.engine)":
                model_path = custom_model_path
            elif selected_model.endswith((".pt", ".onnx", ".torchscript", ".mlpackage", ".engine")):
                model_path = selected_model
            else:
                model_path = custom_model_path

            self.model = YOLO(model_path)
            class_names = list(self.model.names.values())
        self.st.success("Model loaded successfully!")

        selected_classes = self.st.sidebar.multiselect("Classes", class_names, default=class_names)
        self.selected_ind = [class_names.index(option) for option in selected_classes]
        if not isinstance(self.selected_ind, list):
            self.selected_ind = list(self.selected_ind)

    def setup_cameras(self) -> bool:
        """Initialize multiple cameras."""
        self.camera_caps = []
        self.camera_frames = {}
        self.camera_fps = {}
        self.camera_frame_times = {}
        successful_cameras = 0

        for i, cam_idx in enumerate(self.camera_indices):
            try:
                cap = cv2.VideoCapture(cam_idx)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    time.sleep(0.1)

                    test_success = False
                    for _ in range(3):
                        ret, test_frame = cap.read()
                        if ret and test_frame is not None and test_frame.size > 0:
                            test_success = True
                            break
                        time.sleep(0.05)

                    if test_success:
                        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        self.camera_caps.append(cap)
                        self.camera_frames[i] = np.zeros((actual_height, actual_width, 3), dtype=np.uint8)
                        self.camera_fps[i] = 0.0
                        self.camera_frame_times[i] = []
                        successful_cameras += 1
                        LOGGER.info(f"Camera {cam_idx} initialized successfully: {actual_width}x{actual_height}")
                    else:
                        self.camera_caps.append(None)
                        cap.release()
                        LOGGER.warning(f"Camera {cam_idx} opened but cannot read frames")
                else:
                    self.camera_caps.append(None)
                    LOGGER.warning(f"Failed to open camera {cam_idx}")
                time.sleep(0.2)
            except Exception as e:
                self.camera_caps.append(None)
                LOGGER.error(f"Error initializing camera {cam_idx}: {e}")

        return successful_cameras > 0

    def camera_thread(self, camera_index: int, cap: cv2.VideoCapture) -> None:
        """Thread function to capture frames from a single camera."""
        consecutive_failures = 0
        max_failures = 10

        while self.camera_running and cap is not None and cap.isOpened():
            try:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    self.camera_frames[camera_index] = frame.copy()

                    current_time = time.time()
                    if camera_index in self.camera_frame_times:
                        self.camera_frame_times[camera_index].append(current_time)
                        if len(self.camera_frame_times[camera_index]) > 30:
                            self.camera_frame_times[camera_index].pop(0)

                        if len(self.camera_frame_times[camera_index]) > 1:
                            time_diff = self.camera_frame_times[camera_index][-1] - self.camera_frame_times[camera_index][0]
                            if time_diff > 0:
                                self.camera_fps[camera_index] = (len(self.camera_frame_times[camera_index]) - 1) / time_diff

                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= max_failures:
                        LOGGER.error(f"Camera {camera_index} failed max times, stopping thread")
                        break
                time.sleep(0.01)
            except Exception as e:
                consecutive_failures += 1
                LOGGER.error(f"Error in camera thread {camera_index}: {e}")
                if consecutive_failures >= max_failures:
                    break
                time.sleep(0.1)

    def multi_camera_inference(self) -> None:
        """Perform inference on multiple cameras."""
        self.st.info(f"🎥 Attempting to initialize {len(self.camera_indices)} cameras: {self.camera_indices}")

        if not self.setup_cameras():
            self.st.error("❌ Failed to initialize any cameras.")
            return

        working_cameras = sum(1 for cap in self.camera_caps if cap is not None)
        self.st.success(f"✅ {working_cameras} cameras initialized successfully!")

        self.camera_running = True
        self.camera_threads = []

        for i, cap in enumerate(self.camera_caps):
            if cap is not None:
                thread = threading.Thread(target=self.camera_thread, args=(i, cap))
                thread.daemon = True
                thread.start()
                self.camera_threads.append(thread)

        stop_button = self.st.sidebar.button("Stop")

        try:
            inference_start_time = time.time()
            total_frames_processed = 0

            while self.camera_running:
                if stop_button:
                    break

                for cam_idx in range(len(self.camera_caps)):
                    if cam_idx in self.camera_frames and cam_idx in self.camera_containers:
                        frame = self.camera_frames[cam_idx].copy()
                        real_cam_id = self.camera_indices[cam_idx]  # 真实的USB ID (0,2,4,6)

                        if frame is not None and frame.size > 0:
                            if self.enable_trk:
                                results = self.model.track(
                                    frame, conf=self.conf, iou=self.iou, classes=self.selected_ind,
                                    persist=True, imgsz=(480, 640)
                                )
                            else:
                                results = self.model(frame, conf=self.conf, iou=self.iou, classes=self.selected_ind,
                                                     imgsz=(480, 640))

                            annotated_frame = results[0].plot(labels=self.show_class_names)

                            # [新增] 绘制距离和角度
                            if self.show_distance and results[0].boxes is not None:
                                for box in results[0].boxes:
                                    # 获取Box信息
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    class_id = int(box.cls.item())

                                    # 计算
                                    dist, azim, global_ang = self.calculate_spatial_info(
                                        (x1, y1, x2, y2),
                                        class_id,
                                        real_cam_id
                                    )

                                    # 绘制文本 (白色带黑边，确保可见)
                                    text = f"D:{dist:.2f}m A:{azim:.1f}"
                                    text_pos = (int(x1), int(y1) - 10)
                                    if text_pos[1] < 20:
                                        text_pos = (int(x1), int(y2) + 20)

                                    cv2.putText(annotated_frame, text, text_pos, 0, 0.5, (0, 0, 0), 3)  # 黑边
                                    cv2.putText(annotated_frame, text, text_pos, 0, 0.5, (0, 255, 255), 1)  # 黄字

                            # Save detection results if enabled
                            self.save_detection_results(results, frame, real_cam_id)

                            # Update UI
                            org_container, ann_container = self.camera_containers[cam_idx]
                            org_container.image(frame, channels="BGR", caption=f"Camera {real_cam_id} - Original")
                            ann_container.image(annotated_frame, channels="BGR",
                                                caption=f"Camera {real_cam_id} - Predicted")

                            if cam_idx in self.fps_containers:
                                self.fps_containers[cam_idx].metric(
                                    f"Camera {real_cam_id} FPS",
                                    f"{self.camera_fps.get(cam_idx, 0.0):.1f}"
                                )

                            total_frames_processed += 1

                if self.overall_fps_container:
                    elapsed_time = time.time() - inference_start_time
                    if elapsed_time > 0:
                        overall_fps = total_frames_processed / elapsed_time
                        avg_camera_fps = sum(self.camera_fps.values()) / len(
                            self.camera_fps) if self.camera_fps else 0

                        self.overall_fps_container.markdown(f"""
                        ### 📊 Performance Metrics
                        - **Overall Processing FPS**: {overall_fps:.1f}
                        - **Average Camera FPS**: {avg_camera_fps:.1f}
                        - **Active Cameras**: {working_cameras}
                        """)

                time.sleep(0.01)

        finally:
            self.camera_running = False
            for thread in self.camera_threads:
                if thread.is_alive():
                    thread.join(timeout=1.0)
            for cap in self.camera_caps:
                if cap is not None:
                    cap.release()

    def save_detection_results(self, results, frame: np.ndarray, camera_id: int) -> None:
        """Save detection results to specified directory."""
        if not self.save_detections or not results[0].boxes:
            return

        # Console logging for detections
        detected_classes = []
        if results[0].boxes is not None:
            for box in results[0].boxes:
                class_id = int(box.cls.item())
                class_name = self.model.names[class_id]
                detected_classes.append(class_name)

        current_time = time.time()

        # Stats tracking
        if camera_id not in self.class_detection_stats:
            self.class_detection_stats[camera_id] = {}

        for class_name in detected_classes:
            if class_name not in self.class_detection_stats[camera_id]:
                self.class_detection_stats[camera_id][class_name] = 0
            self.class_detection_stats[camera_id][class_name] += 1

        self.last_detection_time[camera_id] = current_time
        # print(f"🎯 Camera {camera_id} - Time: {current_time:.3f} - Detected: {detected_classes}")

        # Directory setup
        if camera_id not in self.detection_count:
            self.detection_count[camera_id] = 0

        camera_dir = Path(self.save_directory) / f"camera_{camera_id}"
        camera_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.detection_count[camera_id] += 1
        base_filename = f"detection_{timestamp}_{self.detection_count[camera_id]:04d}"

        if self.save_images:
            img_path = camera_dir / f"{base_filename}.jpg"
            annotated_frame = results[0].plot(labels=self.show_class_names)
            cv2.imwrite(str(img_path), annotated_frame)

        if self.save_labels:
            label_path = camera_dir / f"{base_filename}.txt"
            self._save_yolo_labels(results[0], label_path, frame.shape, camera_id)

    def _save_yolo_labels(self, result, label_path: Path, img_shape: tuple, cam_id=0) -> None:
        """Save detection results in YOLO format + Distance Info."""
        if not result.boxes:
            return

        height, width = img_shape[:2]

        with open(label_path, 'w') as f:
            for box in result.boxes:
                class_id = int(box.cls.item())
                confidence = float(box.conf.item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # YOLO Standard Format
                x_center = (x1 + x2) / 2 / width
                y_center = (y1 + y2) / 2 / height
                bbox_width = (x2 - x1) / width
                bbox_height = (y2 - y1) / height

                # [新增] 计算距离并在Log中保存
                dist, azim, global_ang = self.calculate_spatial_info((x1, y1, x2, y2), class_id, cam_id)

                # 扩展保存格式: class x y w h conf dist azimuth
                f.write(
                    f"{class_id} {x_center:.6f} {y_center:.6f} {bbox_width:.6f} {bbox_height:.6f} {confidence:.6f} {dist:.3f} {azim:.1f}\n")

    def image_inference(self) -> None:
        """Perform inference on uploaded images."""
        for idx, img_info in enumerate(self.img_file_names):
            img_path = img_info["path"]
            image = cv2.imread(img_path)
            if image is not None:
                self.st.markdown(f"#### Processed: {img_info['name']}")
                col1, col2 = self.st.columns(2)
                with col1:
                    self.st.image(image, channels="BGR", caption="Original Image")
                results = self.model(image, conf=self.conf, iou=self.iou, classes=self.selected_ind, imgsz=(480, 640))
                annotated_image = results[0].plot(labels=self.show_class_names)

                # [新增] 绘制距离 (假设图片为 Cam 0)
                if self.show_distance and results[0].boxes is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        class_id = int(box.cls.item())
                        dist, azim, _ = self.calculate_spatial_info((x1, y1, x2, y2), class_id, 0)
                        text = f"D:{dist:.2f}m"
                        cv2.putText(annotated_image, text, (int(x1), int(y1) - 10), 0, 0.5, (0, 255, 255), 2)

                with col2:
                    self.st.image(annotated_image, channels="BGR", caption="Predicted Image")
                try:
                    os.unlink(img_path)
                except FileNotFoundError:
                    pass
            else:
                self.st.error("Could not load the uploaded image.")

    def inference(self) -> None:
        """Perform real-time object detection inference."""
        self.web_ui()
        self.sidebar()
        self.source_upload()
        self.configure()

        if self.st.sidebar.button("Start"):
            if self.source == "image":
                if self.img_file_names:
                    self.image_inference()
                else:
                    self.st.info("Please upload an image file.")
                return
            elif self.source == "multi-webcam":
                self.multi_camera_inference()
                return

            stop_button = self.st.sidebar.button("Stop")

            if self.source == "webcam":
                cap = cv2.VideoCapture(self.vid_file_name, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap = cv2.VideoCapture(self.vid_file_name)

                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    self.st.info(f"Camera resolution set to: {actual_width}x{actual_height}")
            else:
                cap = cv2.VideoCapture(self.vid_file_name)

            if not cap.isOpened():
                self.st.error("Could not open webcam or video source.")
                return

            while cap.isOpened():
                success, frame = cap.read()
                if not success:
                    break

                if self.enable_trk:
                    results = self.model.track(
                        frame, conf=self.conf, iou=self.iou, classes=self.selected_ind,
                        persist=True, imgsz=(480, 640)
                    )
                else:
                    results = self.model(frame, conf=self.conf, iou=self.iou, classes=self.selected_ind,
                                         imgsz=(480, 640))

                annotated_frame = results[0].plot(labels=self.show_class_names)

                # [新增] 单摄像头模式也绘制距离
                if self.show_distance and results[0].boxes is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        class_id = int(box.cls.item())
                        dist, azim, _ = self.calculate_spatial_info((x1, y1, x2, y2), class_id, 0)

                        text = f"D:{dist:.2f}m"
                        cv2.putText(annotated_frame, text, (int(x1), int(y1) - 10), 0, 0.5, (0, 0, 0), 3)
                        cv2.putText(annotated_frame, text, (int(x1), int(y1) - 10), 0, 0.5, (0, 255, 255), 1)

                # Save results (using ID 0 for single source)
                self.save_detection_results(results, frame, 0)

                if stop_button:
                    cap.release()
                    self.st.stop()

                self.org_frame.image(frame, channels="BGR", caption="Original Frame")
                self.ann_frame.image(annotated_frame, channels="BGR", caption="Predicted Frame")

            cap.release()


if __name__ == "__main__":
    args = len(sys.argv)
    model = sys.argv[1] if args > 1 else None
    Inference(model=model).inference()