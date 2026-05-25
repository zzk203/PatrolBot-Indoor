# 07. 摄像头缓存 (CameraBuffer)

> 对应需求: FR-16 巡逻点拍照
> 参考设计: detailed_design.md §2.7

## 7.1 头文件

- [ ] 创建 `include/patrol_bot/camera_buffer.hpp`
- [ ] 声明 `CameraBuffer` 类
- [ ] 声明构造函数 `CameraBuffer(node, topic, save_dir, format)`
- [ ] 声明 `bool save_latest(const std::string& filename)`
- [ ] 声明 `bool has_frame() const`
- [ ] 声明 `void image_callback()` 私有方法
- [ ] 声明成员变量：`sub_`、`save_dir_`、`format_`、`latest_frame_`、`has_frame_`、`mutex_`

## 7.2 实现

- [ ] 创建 `src/camera_buffer.cpp`
- [ ] 实现构造函数：
  - 保存 save_dir_ / format_
  - `std::filesystem::create_directories(save_dir)` 创建保存目录
  - 创建 Subscription `<sensor_msgs::msg::Image>`

### 7.2.1 图像回调

- [ ] 实现 `image_callback(msg)`
- [ ] 使用 `cv_bridge::toCvCopy(msg, "bgr8")` 将 ROS Image 转为 cv::Mat
- [ ] `std::lock_guard<std::mutex>` 保护帧缓存更新
- [ ] `latest_frame_ = crop(); has_frame_ = true`

### 7.2.2 保存与查询

- [ ] 实现 `has_frame()`：`std::lock_guard` 保护，返回 `has_frame_`
- [ ] 实现 `save_latest(filename)`：
  - `std::lock_guard` 保护读取
  - 无可用帧时返回 false
  - 拼接路径：`save_dir_ + "/" + filename + "." + format_`
  - `cv::imwrite(path, latest_frame_)`
  - 返回 true

## 7.3 编译验证

- [ ] 编译通过，依赖 OpenCV、cv_bridge 正确链接
- [ ] 验证无帧时 save_latest 返回 false
- [ ] 验证保存的图片文件存在且格式正确
