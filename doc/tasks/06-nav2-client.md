# 06. 导航客户端 (Nav2ActionClient)

> 对应需求: FR-02 巡逻点到达、FR-06 低电回充导航、FR-14 自动绕行、FR-15 超时跳过
> 参考设计: detailed_design.md §2.6

## 6.1 头文件

- [ ] 创建 `include/patrol_bot/nav2_action_client.hpp`
- [ ] 定义 `NavResult` 枚举：SUCCESS / FAILURE / RUNNING / NOT_STARTED / ERROR
- [ ] 声明 `Nav2ActionClient` 类
- [ ] 声明构造函数 `Nav2ActionClient(rclcpp::Node* node, const std::string& action_name = "/navigate_to_pose")`
- [ ] 声明 `bool wait_for_server(std::chrono::seconds timeout)`
- [ ] 声明 `void send_goal(double x, double y, double yaw)`
- [ ] 声明 `NavResult check_result()`
- [ ] 声明 `void cancel_goal()`
- [ ] 声明 `bool is_navigating() const`
- [ ] 声明成员变量：`node_`、`client_`、`goal_handle_`
- [ ] 声明 atomic 变量：`result_`、`result_ready_`、`server_ready_`

## 6.2 实现

- [ ] 创建 `src/nav2_action_client.cpp`
- [ ] 实现构造函数：创建 Action Client

### 6.2.1 等待服务器

- [ ] 实现 `wait_for_server(timeout)`
- [ ] 调用 `client_->wait_for_action_server(timeout)` 等待就绪
- [ ] 设置 `server_ready_` 标志

### 6.2.2 发送导航目标

- [ ] 实现 `send_goal(x, y, yaw)`
- [ ] 服务器未就绪时返回 ERROR
- [ ] 构造 `NavigateToPose::Goal` 消息：
  - `pose.header.frame_id = "map"`
  - `pose.header.stamp = node_->now()`
  - `pose.pose.position = (x, y)`
  - `pose.pose.orientation = quaternion_from_yaw(yaw)`
- [ ] 实现 yaw → quaternion 转换（使用 `tf2::Quaternion` 或手动计算）
- [ ] 注册 `result_callback`：根据结果码设置 `result_` = SUCCESS / FAILURE
- [ ] 注册 `goal_response_callback`：goal 未被接受时设置 result_ = ERROR
- [ ] 调用 `client_->async_send_goal()`

### 6.2.3 结果检查与取消

- [ ] 实现 `check_result()`：
  - `result_ready_ == true` → 返回 `result_`
  - 否则 → 返回 RUNNING
- [ ] 实现 `cancel_goal()`：
  - 如果有 `goal_handle_`，调用 `client_->async_cancel_goal()`
  - 重置 `result_ = NOT_STARTED`
  - 重置 `result_ready_ = false`
- [ ] 实现 `is_navigating()`：返回 `goal_handle_ != nullptr`

## 6.3 编译验证

- [ ] 编译通过，依赖 nav2_msgs、rclcpp_action 正确链接
- [ ] 验证 send_goal / check_result / cancel_goal 状态机流程
- [ ] 验证 yaw 到 quaternion 转换正确性
