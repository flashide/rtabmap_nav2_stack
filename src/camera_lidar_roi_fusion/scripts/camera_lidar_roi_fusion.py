#!/usr/bin/env python3
"""ROI-based camera-lidar fusion."""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, PointCloud2, RegionOfInterest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from camera_lidar_roi_fusion.msg import FusedDetection, FusedDetectionArray


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def bbox_center_xy(center) -> Tuple[float, float]:
    if hasattr(center, 'position'):
        return float(center.position.x), float(center.position.y)
    return float(center.x), float(center.y)


def hypothesis_label_and_score(detection) -> Tuple[str, float]:
    if not detection.results:
        return '', 0.0
    result = detection.results[0]
    hypothesis = result.hypothesis if hasattr(result, 'hypothesis') else result
    class_id = str(getattr(hypothesis, 'class_id', ''))
    score = float(getattr(result, 'score', getattr(hypothesis, 'score', 0.0)))
    return class_id, score


class CameraLidarRoiFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_lidar_roi_fusion')

        self.declare_parameter('pointcloud_topic', '/cloud_registered_body')
        self.declare_parameter('camera_info_topic', '/sensors/camera/rgb/camera_info')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('fused_detections_topic', '/perception/fused_detections')
        self.declare_parameter('fused_cloud_topic', '/perception/fused_cloud')
        self.declare_parameter('marker_topic', '/perception/fusion_markers')
        self.declare_parameter('min_depth_m', 0.5)
        self.declare_parameter('max_depth_m', 60.0)
        self.declare_parameter('distance_tolerance_m', 1.5)
        self.declare_parameter('depth_quantile', 0.35)
        self.declare_parameter('min_points_per_roi', 3)
        self.declare_parameter('sync_slop_s', 0.25)

        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance_m').value)
        self.depth_quantile = float(self.get_parameter('depth_quantile').value)
        self.min_points_per_roi = int(self.get_parameter('min_points_per_roi').value)
        self.sync_slop = float(self.get_parameter('sync_slop_s').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_detections: Optional[Detection2DArray] = None

        self.fused_pub = self.create_publisher(
            FusedDetectionArray, self.get_parameter('fused_detections_topic').value, 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('fused_cloud_topic').value, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_callback, 10)
        self.create_subscription(Detection2DArray, self.detections_topic, self._detections_callback, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._pointcloud_callback, 10)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _detections_callback(self, msg: Detection2DArray) -> None:
        self.latest_detections = msg

    def _pointcloud_callback(self, cloud_msg: PointCloud2) -> None:
        if self.latest_camera_info is None or self.latest_detections is None:
            return

        camera_info = self.latest_camera_info
        detections = self.latest_detections
        if abs(stamp_to_seconds(cloud_msg.header.stamp) - stamp_to_seconds(detections.header.stamp)) > self.sync_slop:
            return

        camera_frame = camera_info.header.frame_id or 'camera_link'
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warning(f'Cannot transform {cloud_msg.header.frame_id} -> {camera_frame}: {exc}')
            return

        xyz_cloud = self._read_xyz_points(cloud_msg)
        if xyz_cloud.shape[0] == 0:
            return

        xyz_camera = self._transform_points(xyz_cloud, transform)
        uv, valid_mask = self._project_points(xyz_camera, camera_info)
        if not np.any(valid_mask):
            return

        fused_msg = FusedDetectionArray()
        fused_msg.header = detections.header
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker(cloud_msg.header.frame_id, detections.header.stamp))
        kept_points: List[Tuple[float, float, float]] = []
        marker_id = 0

        for index, detection in enumerate(detections.detections):
            roi = self._roi_from_detection(detection, camera_info.width, camera_info.height)
            if roi.width == 0 or roi.height == 0:
                continue

            in_roi = self._points_in_roi(uv, roi) & valid_mask
            roi_indices = np.nonzero(in_roi)[0]
            if roi_indices.size < self.min_points_per_roi:
                continue

            near_depth = float(np.quantile(xyz_camera[roi_indices, 2], self.depth_quantile))
            refined_mask = in_roi & (xyz_camera[:, 2] <= near_depth + self.distance_tolerance)
            refined_indices = np.nonzero(refined_mask)[0]
            if refined_indices.size < self.min_points_per_roi:
                refined_indices = roi_indices

            refined_points_cloud = xyz_cloud[refined_indices]
            refined_points_camera = xyz_camera[refined_indices]
            kept_points.extend([tuple(point) for point in refined_points_cloud.tolist()])

            distance = float(np.median(refined_points_camera[:, 2]))
            centroid = np.mean(refined_points_cloud, axis=0)
            class_id, score = hypothesis_label_and_score(detection)

            fused_detection = FusedDetection()
            fused_detection.class_id = class_id
            fused_detection.score = score
            fused_detection.distance = distance
            fused_detection.center = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
            fused_detection.roi = roi
            fused_msg.detections.append(fused_detection)

            box_min = np.min(refined_points_cloud, axis=0)
            box_max = np.max(refined_points_cloud, axis=0)
            markers.markers.extend(self._make_markers(
                marker_id,
                cloud_msg.header.frame_id,
                detections.header.stamp,
                centroid,
                box_min,
                box_max,
                distance,
                class_id or f'roi_{index}',
            ))
            marker_id += 2

        if not fused_msg.detections:
            self.marker_pub.publish(markers)
            return

        self.fused_pub.publish(fused_msg)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(cloud_msg.header, kept_points))
        self.marker_pub.publish(markers)

    def _read_xyz_points(self, cloud_msg: PointCloud2) -> np.ndarray:
        points = [
            [float(x), float(y), float(z)]
            for x, y, z in point_cloud2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
        ]
        if not points:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(points, dtype=np.float64)

    def _transform_points(self, xyz_points: np.ndarray, transform) -> np.ndarray:
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        rotation = quaternion_to_rotation_matrix(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        return (rotation @ xyz_points.T).T + translation

    def _project_points(self, xyz_camera: np.ndarray, camera_info: CameraInfo) -> Tuple[np.ndarray, np.ndarray]:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        z = xyz_camera[:, 2]
        valid = np.isfinite(z) & (z >= self.min_depth) & (z <= self.max_depth)
        uv = np.zeros((xyz_camera.shape[0], 2), dtype=np.float64)
        uv[valid, 0] = fx * xyz_camera[valid, 0] / z[valid] + cx
        uv[valid, 1] = fy * xyz_camera[valid, 1] / z[valid] + cy
        valid &= (
            (uv[:, 0] >= 0.0) & (uv[:, 0] < float(camera_info.width)) &
            (uv[:, 1] >= 0.0) & (uv[:, 1] < float(camera_info.height))
        )
        return uv, valid

    def _roi_from_detection(self, detection, width_limit: int, height_limit: int) -> RegionOfInterest:
        cx, cy = bbox_center_xy(detection.bbox.center)
        width = max(0.0, float(detection.bbox.size_x))
        height = max(0.0, float(detection.bbox.size_y))
        x_offset = int(max(0.0, cx - width * 0.5))
        y_offset = int(max(0.0, cy - height * 0.5))
        x_max = int(min(float(width_limit), cx + width * 0.5))
        y_max = int(min(float(height_limit), cy + height * 0.5))

        roi = RegionOfInterest()
        roi.x_offset = x_offset
        roi.y_offset = y_offset
        roi.width = max(0, x_max - x_offset)
        roi.height = max(0, y_max - y_offset)
        roi.do_rectify = False
        return roi

    def _points_in_roi(self, uv: np.ndarray, roi: RegionOfInterest) -> np.ndarray:
        return (
            (uv[:, 0] >= float(roi.x_offset)) &
            (uv[:, 0] < float(roi.x_offset + roi.width)) &
            (uv[:, 1] >= float(roi.y_offset)) &
            (uv[:, 1] < float(roi.y_offset + roi.height))
        )

    def _make_markers(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        centroid: Sequence[float],
        box_min: Sequence[float],
        box_max: Sequence[float],
        distance: float,
        label: str,
    ) -> List[Marker]:
        box_marker = Marker()
        box_marker.header.frame_id = frame_id
        box_marker.header.stamp = stamp
        box_marker.ns = 'camera_lidar_roi_fusion'
        box_marker.id = marker_id
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        box_marker.pose.position.x = float((box_min[0] + box_max[0]) * 0.5)
        box_marker.pose.position.y = float((box_min[1] + box_max[1]) * 0.5)
        box_marker.pose.position.z = float((box_min[2] + box_max[2]) * 0.5)
        box_marker.pose.orientation.w = 1.0
        box_marker.scale.x = max(0.1, float(box_max[0] - box_min[0]))
        box_marker.scale.y = max(0.1, float(box_max[1] - box_min[1]))
        box_marker.scale.z = max(0.1, float(box_max[2] - box_min[2]))
        box_marker.color = ColorRGBA(r=0.0, g=0.9, b=0.3, a=0.35)

        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = stamp
        text_marker.ns = 'camera_lidar_roi_fusion'
        text_marker.id = marker_id + 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(centroid[0])
        text_marker.pose.position.y = float(centroid[1])
        text_marker.pose.position.z = float(centroid[2] + 0.4)
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.25
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.text = f'{label}: {distance:.2f} m'
        return [box_marker, text_marker]

    def _delete_all_marker(self, frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraLidarRoiFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
