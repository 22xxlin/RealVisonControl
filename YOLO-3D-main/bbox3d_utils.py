import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter
from collections import defaultdict
import math

# Default camera intrinsic matrix
DEFAULT_K = np.array([
    [718.856, 0.0, 607.1928],
    [0.0, 718.856, 185.2157],
    [0.0, 0.0, 1.0]
])

# Default camera projection matrix
DEFAULT_P = np.array([
    [718.856, 0.0, 607.1928, 45.38225],
    [0.0, 718.856, 185.2157, -0.1130887],
    [0.0, 0.0, 1.0, 0.003779761]
])

# Average dimensions (meters)
DEFAULT_DIMS = {
    'car': np.array([1.52, 1.64, 3.85]),
    'truck': np.array([3.07, 2.63, 11.17]),
    'bus': np.array([3.07, 2.63, 11.17]),
    'motorcycle': np.array([1.50, 0.90, 2.20]),
    'bicycle': np.array([1.40, 0.70, 1.80]),
    'person': np.array([1.75, 0.60, 0.60]),
    'dog': np.array([0.80, 0.50, 1.10]),
    'cat': np.array([0.40, 0.30, 0.70]),
    'potted plant': np.array([0.80, 0.40, 0.40]),
    'plant': np.array([0.80, 0.40, 0.40]),
    'chair': np.array([0.80, 0.60, 0.60]),
    'sofa': np.array([0.80, 0.85, 2.00]),
    'table': np.array([0.75, 1.20, 1.20]),
    'bed': np.array([0.60, 1.50, 2.00]),
    'tv': np.array([0.80, 0.15, 1.20]),
    'laptop': np.array([0.02, 0.25, 0.35]),
    'keyboard': np.array([0.03, 0.15, 0.45]),
    'mouse': np.array([0.03, 0.06, 0.10]),
    'book': np.array([0.03, 0.20, 0.15]),
    'bottle': np.array([0.25, 0.10, 0.10]),
    'cup': np.array([0.10, 0.08, 0.08]),
    'vase': np.array([0.30, 0.15, 0.15])
}

class BBox3DEstimator:
    def __init__(self, camera_matrix=None, projection_matrix=None, class_dims=None):
        self.K = camera_matrix if camera_matrix is not None else DEFAULT_K
        self.P = projection_matrix if projection_matrix is not None else DEFAULT_P
        self.dims = class_dims if class_dims is not None else DEFAULT_DIMS
        self.kf_trackers = {}
        self.box_history = defaultdict(list)
        self.max_history = 5
    
    def estimate_3d_box(self, bbox_2d, depth_value, class_name, object_id=None):
        x1, y1, x2, y2 = bbox_2d
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        width_2d = x2 - x1
        height_2d = y2 - y1
        
        if class_name.lower() in self.dims:
            dimensions = self.dims[class_name.lower()].copy()
        else:
            dimensions = self.dims['car'].copy()
        
        # Adjust dimensions logic
        if 'plant' in class_name.lower() or 'potted plant' in class_name.lower():
            dimensions[0] = height_2d / 120
            dimensions[1] = dimensions[0] * 0.6
            dimensions[2] = dimensions[0] * 0.6
        elif 'person' in class_name.lower():
            dimensions[0] = height_2d / 100
            dimensions[1] = dimensions[0] * 0.3
            dimensions[2] = dimensions[0] * 0.3
        
        # Map depth_value (0-1) to visualization distance range
        distance = 1.0 + depth_value * 9.0
        location = self._backproject_point(center_x, center_y, distance)
        
        if 'plant' in class_name.lower() or 'potted plant' in class_name.lower():
            bottom_y = y2
            location[1] = self._backproject_point(center_x, bottom_y, distance)[1]
        
        orientation = self._estimate_orientation(bbox_2d, location, class_name)
        
        box_3d = {
            'dimensions': dimensions,
            'location': location,
            'orientation': orientation,
            'bbox_2d': bbox_2d,
            'object_id': object_id,
            'class_name': class_name
        }
        
        if object_id is not None:
            box_3d = self._apply_kalman_filter(box_3d, object_id)
            self.box_history[object_id].append(box_3d)
            if len(self.box_history[object_id]) > self.max_history:
                self.box_history[object_id].pop(0)
            box_3d = self._apply_temporal_filter(object_id)
        
        return box_3d
    
    def _backproject_point(self, x, y, depth):
        point_2d = np.array([x, y, 1.0])
        point_3d = np.linalg.inv(self.K) @ point_2d * depth
        point_3d[1] = point_3d[1] * 0.5
        return point_3d
    
    def _estimate_orientation(self, bbox_2d, location, class_name):
        theta_ray = np.arctan2(location[0], location[2])
        if 'plant' in class_name.lower() or 'potted plant' in class_name.lower():
            return theta_ray
        if 'person' in class_name.lower():
            alpha = 0.0
        else:
            x1, y1, x2, y2 = bbox_2d
            width = x2 - x1
            height = y2 - y1
            aspect_ratio = width / height if height > 0 else 1.0
            image_center_x = self.K[0, 2]
            if aspect_ratio > 1.5:
                if (x1 + x2) / 2 < image_center_x:
                    alpha = np.pi / 2
                else:
                    alpha = -np.pi / 2
            else:
                alpha = 0.0
        return alpha + theta_ray
    
    def _init_kalman_filter(self, box_3d):
        kf = KalmanFilter(dim_x=11, dim_z=7)
        kf.x = np.array([
            box_3d['location'][0], box_3d['location'][1], box_3d['location'][2],
            box_3d['dimensions'][1], box_3d['dimensions'][0], box_3d['dimensions'][2],
            box_3d['orientation'], 0, 0, 0, 0
        ])
        dt = 1.0
        kf.F = np.eye(11)
        kf.F[0, 7] = dt; kf.F[1, 8] = dt; kf.F[2, 9] = dt; kf.F[6, 10] = dt
        kf.H = np.zeros((7, 11))
        kf.H[0, 0] = 1; kf.H[1, 1] = 1; kf.H[2, 2] = 1; kf.H[3, 3] = 1
        kf.H[4, 4] = 1; kf.H[5, 5] = 1; kf.H[6, 6] = 1
        kf.R = np.eye(7) * 0.1
        kf.R[0:3, 0:3] *= 1.0; kf.R[3:6, 3:6] *= 0.1; kf.R[6, 6] = 0.3
        kf.Q = np.eye(11) * 0.1
        kf.Q[7:11, 7:11] *= 0.5
        kf.P = np.eye(11) * 1.0
        kf.P[7:11, 7:11] *= 10.0
        return kf
    
    def _apply_kalman_filter(self, box_3d, object_id):
        if object_id not in self.kf_trackers:
            self.kf_trackers[object_id] = self._init_kalman_filter(box_3d)
        kf = self.kf_trackers[object_id]
        kf.predict()
        measurement = np.array([
            box_3d['location'][0], box_3d['location'][1], box_3d['location'][2],
            box_3d['dimensions'][1], box_3d['dimensions'][0], box_3d['dimensions'][2],
            box_3d['orientation']
        ])
        kf.update(measurement)
        filtered_box = box_3d.copy()
        filtered_box['location'] = np.array([kf.x[0], kf.x[1], kf.x[2]])
        filtered_box['dimensions'] = np.array([kf.x[4], kf.x[3], kf.x[5]])
        filtered_box['orientation'] = kf.x[6]
        return filtered_box
    
    def _apply_temporal_filter(self, object_id):
        history = self.box_history[object_id]
        if len(history) < 2: return history[-1]
        current_box = history[-1]
        alpha = 0.7
        filtered_box = current_box.copy()
        for i in range(len(history) - 2, -1, -1):
            weight = alpha * (1 - alpha) ** (len(history) - i - 2)
            filtered_box['location'] = filtered_box['location'] * (1 - weight) + history[i]['location'] * weight
            angle_diff = history[i]['orientation'] - filtered_box['orientation']
            if angle_diff > np.pi: angle_diff -= 2 * np.pi
            elif angle_diff < -np.pi: angle_diff += 2 * np.pi
            filtered_box['orientation'] += angle_diff * weight
        return filtered_box

    def draw_box_3d(self, image, box_3d, color=(0, 255, 0), thickness=2):
        x1, y1, x2, y2 = [int(coord) for coord in box_3d['bbox_2d']]
        depth_value = box_3d.get('depth_value', 0.5)
        width = x2 - x1
        height = y2 - y1
        offset_factor = 1.0 - depth_value
        offset_x = int(width * 0.3 * offset_factor)
        offset_y = int(height * 0.3 * offset_factor)
        offset_x = max(15, min(offset_x, 50))
        offset_y = max(15, min(offset_y, 50))
        
        front_tl = (x1, y1); front_tr = (x2, y1); front_br = (x2, y2); front_bl = (x1, y2)
        back_tl = (x1 + offset_x, y1 - offset_y)
        back_tr = (x2 + offset_x, y1 - offset_y)
        back_br = (x2 + offset_x, y2 - offset_y)
        back_bl = (x1 + offset_x, y2 - offset_y)
        
        overlay = image.copy()
        cv2.rectangle(image, front_tl, front_br, color, thickness)
        cv2.line(image, front_tl, back_tl, color, thickness)
        cv2.line(image, front_tr, back_tr, color, thickness)
        cv2.line(image, front_br, back_br, color, thickness)
        cv2.line(image, front_bl, back_bl, color, thickness)
        cv2.line(image, back_tl, back_tr, color, thickness)
        cv2.line(image, back_tr, back_br, color, thickness)
        cv2.line(image, back_br, back_bl, color, thickness)
        cv2.line(image, back_bl, back_tl, color, thickness)
        
        pts_top = np.array([front_tl, front_tr, back_tr, back_tl], np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(overlay, [pts_top], color)
        pts_right = np.array([front_tr, front_br, back_br, back_tr], np.int32).reshape((-1, 1, 2))
        right_color = (int(color[0] * 0.7), int(color[1] * 0.7), int(color[2] * 0.7))
        cv2.fillPoly(overlay, [pts_right], right_color)
        cv2.addWeighted(overlay, 0.3, image, 0.7, 0, image)
        
        # --- 显示文字 (底部) ---
        class_name = box_3d['class_name']
        obj_id = box_3d.get('object_id')
        text_y = y2 + 20
        line_height = 20
        
        if obj_id is not None:
            cv2.putText(image, f"ID:{obj_id}", (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            text_y += line_height
        
        cv2.putText(image, class_name, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        text_y += line_height
        
        if 'depth_method' in box_3d:
            # 允许任何字符串显示
            cv2.putText(image, str(box_3d['depth_method']), (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            text_y += line_height
        
        # Ground line
        ground_y = y2 + int(height * 0.2)
        cv2.line(image, (int((x1 + x2)/2), y2), (int((x1 + x2)/2), ground_y), color, thickness)
        cv2.circle(image, (int((x1 + x2)/2), ground_y), thickness * 2, color, -1)
        
        return image

    def cleanup_trackers(self, active_ids):
        active_ids_set = set(active_ids)
        for obj_id in list(self.kf_trackers.keys()):
            if obj_id not in active_ids_set: del self.kf_trackers[obj_id]
        for obj_id in list(self.box_history.keys()):
            if obj_id not in active_ids_set: del self.box_history[obj_id]

class BirdEyeView:
    def __init__(self, size=(400, 400), scale=30, camera_height=1.2):
        self.width, self.height = size
        self.scale = scale
        self.camera_height = camera_height
        self.bev_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.origin_x = self.width // 2
        self.origin_y = self.height - 50
    
    def reset(self):
        self.bev_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.bev_image[:, :] = (20, 20, 20)
        grid_spacing = max(int(self.scale), 20)
        for y in range(self.origin_y, 0, -grid_spacing):
            cv2.line(self.bev_image, (0, y), (self.width, y), (50, 50, 50), 1)
        for x in range(0, self.width, grid_spacing):
            cv2.line(self.bev_image, (x, 0), (x, self.height), (50, 50, 50), 1)
        
        axis_length = min(80, self.height // 5)
        cv2.line(self.bev_image, (self.origin_x, self.origin_y), (self.origin_x, self.origin_y - axis_length), (0, 200, 0), 2)
        cv2.line(self.bev_image, (self.origin_x, self.origin_y), (self.origin_x + axis_length, self.origin_y), (0, 0, 200), 2)
        cv2.putText(self.bev_image, "X", (self.origin_x - 15, self.origin_y - axis_length + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
        cv2.putText(self.bev_image, "Y", (self.origin_x + axis_length - 15, self.origin_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 1)
        
        for dist in [1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]:
            y = self.origin_y - int(dist * self.scale)
            if y < 20: continue
            thickness = 2 if dist.is_integer() else 1
            cv2.line(self.bev_image, (self.origin_x - 5, y), (self.origin_x + 5, y), (120, 120, 120), thickness)
            if dist.is_integer():
                cv2.putText(self.bev_image, f"{int(dist)}m", (self.origin_x + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

    def draw_box(self, box_3d, color=None):
        try:
            class_name = box_3d['class_name'].lower()
            depth_value = box_3d.get('depth_value', 0.5)
            depth = 1.0 + depth_value * 4.0
            if 'bbox_2d' in box_3d:
                x1, y1, x2, y2 = box_3d['bbox_2d']
                width_2d = x2 - x1
                size_factor = max(0.5, min(width_2d / 100, 2.0))
            else: size_factor = 1.0
            
            if color is None:
                if 'car' in class_name or 'vehicle' in class_name: color = (0, 0, 255)
                elif 'truck' in class_name or 'bus' in class_name: color = (0, 165, 255)
                elif 'person' in class_name: color = (0, 255, 0)
                else: color = (255, 255, 255)
            
            bev_y = self.origin_y - int(depth * self.scale)
            if 'bbox_2d' in box_3d:
                center_x_2d = (x1 + x2) / 2
                rel_x = (center_x_2d / self.width) - 0.5 # Approximate visual width
                # This width mapping is approximate for visualization
                bev_x = self.origin_x + int(rel_x * self.width * 0.6) 
            else: bev_x = self.origin_x
            
            bev_x = max(20, min(bev_x, self.width - 20))
            bev_y = max(20, min(bev_y, self.origin_y - 10))
            
            radius = int(8 * size_factor)
            cv2.circle(self.bev_image, (bev_x, bev_y), radius, color, -1)
            
            if 'object_id' in box_3d and box_3d['object_id'] is not None:
                cv2.putText(self.bev_image, f"{box_3d['object_id']}", (bev_x - 5, bev_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            cv2.line(self.bev_image, (self.origin_x, self.origin_y), (bev_x, bev_y), (70, 70, 70), 1)
        except Exception as e:
            print(f"Error drawing box in BEV: {e}")

    def get_image(self):
        return self.bev_image