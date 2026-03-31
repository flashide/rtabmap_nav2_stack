# camera_lidar_roi_fusion

轻量版相机 ROI 与激光雷达投影融合节点，思路参考 Autoware `autoware_image_projection_based_fusion` 里的 `roi_pointcloud_fusion`。

## 输入

- `vision_msgs/Detection2DArray`
- `sensor_msgs/CameraInfo`
- `sensor_msgs/PointCloud2`

## 输出

- `camera_lidar_roi_fusion/msg/FusedDetectionArray`
- 筛选后的 `PointCloud2`
- `MarkerArray`

## 启动

```bash
ros2 launch camera_lidar_roi_fusion camera_lidar_roi_fusion.launch.py \
  pointcloud_topic:=/cloud_registered_body \
  camera_info_topic:=/sensors/camera/rgb/camera_info \
  detections_topic:=/detections
```
