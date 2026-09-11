# SysNav 仿真 Episode 日志规范

## 适用范围

每次运行仿真 episode 都必须记录完整的目标导航闭环。日志用于判断目标在哪一层被发现、处理、确认、接管，以及 episode 最终为什么结束。

## 仿真启动要求

- 每次拉起 simulation 时都必须同时拉起 RViz，包括正式 demo、调试、验证、失败重试和短时检查；不得只启动 ROS/Unity 仿真而不启动 RViz。
- RViz 必须是仿真启动流程的默认必需组件，不得设计成需要额外传入 `--rviz` 才会启用的可选项。
- 启动 harness/runner 负责拉起和管理 RViz；logger 只能观察和记录，不得负责启动、停止或控制仿真与 RViz。
- RViz 必须与该次 simulation 使用相同的 ROS domain、workspace 环境和 episode 生命周期，其原始输出统一写入该 episode 的 `debug.log`。
- 启动后必须确认 RViz 进程仍在运行。若 RViz 启动失败或提前退出，本次 simulation 启动不完整，不得作为有效 demo；必须在 `metrics.json` 中记录 `rviz_started: false`、`success: false` 和实际失败原因。
- planner 确认到达目标后，录制必须继续保留 2 秒画面，再由启动 harness/runner 结束 episode；logger 只能记录到达事件，不得触发该计时或停止仿真。

## Demo 画面要求

- `demo.mp4` 必须同时包含三个与本 episode 实时同步的视图：带 YOLO 框和实例 mask 的 panorama、以 RViz 俯瞰图为背景的 object-memory 生长图、Unity 第三人称视角。
- panorama 必须是面积最大的主视图，并完整保留检测框、类别/track 标注和实例 mask；其余两个视图不得遮挡 panorama 中的关键目标证据。
- `demo.mp4` 必须由专用合成器直接将 panorama、RViz render stream 和 Unity render stream 编码到同一画布；禁止使用整个桌面或显示器的 screen recording 作为最终视频源，最终画面不得包含桌面栏、其他应用窗口或窗口排列产生的空白区域。
- RViz 必须使用单一、放大的俯瞰 render view，不得因残留 Image dock、工具面板或多个小视窗而缩成缩略图；其视野必须覆盖已探索楼层，以累积方式保留 room/object memory 的新建、更新和合并过程。
- Unity 作为辅助视图可小于 RViz memory 俯瞰图，但仍必须是连续、实时的第三人称视角。
- `demo.mp4` 必须包含实时数据面板，至少显示当前 pipeline stage、目标类别、memory object 数量，以及最新一次 VLM 请求的类型、输入和输出；这些内容必须来自当前 episode 的实时 ROS 消息/事件，不得事后伪造。
- 每收到一次 VLM 输出，视频中都必须立即出现醒目的 `VLM OUTPUT RECEIVED` caption，包含 request kind、object/request ID 和结果摘要；同时返回多个结果时必须逐条排队展示，不得被后一条瞬间覆盖。
- object-memory 生长图必须叠加在 RViz 的真实俯瞰地图、点云或占据区域上，实时展示 memory object 的创建、更新、合并及空间位置；不得使用空白背景，也不得用 episode 结束后的离线路径图或静态截图冒充实时 memory 生长。
- Unity 视图必须使用能同时看见机器人和周围环境的第三人称相机，不得以第一人称 panorama 或静态 Unity 截图代替。
- 三个视图必须来自同一次 simulation，使用同一 ROS clock 或保留可验证的时间对应关系；禁止拼接其他 episode、预录素材或不同步的画面。
- `demo-preview.jpg` 必须从最终 `demo.mp4` 的实际帧中提取，并尽量同时展示上述三个视图和目标检测证据。
- 任一必需视图缺失、冻结或录制失败时，不得将 demo 标记为有效；必须在 `metrics.json` 中分别记录 `panorama_recorded`、`rviz_memory_view_recorded`、`unity_third_person_recorded` 及失败原因。
- 实时面板未录制、未收到本轮数据，或已有 VLM 输出却未产生 caption 时，同样不得将 demo 标记为有效；必须记录 `data_panel_recorded`、`vlm_outputs_received` 和 `vlm_result_captions_shown`。

## 每次 episode 必须生成的文件

每个 episode 必须使用独立目录 `recordings/<run-name>/`。该 episode 的日志、图片、视频和中间证据只能放在该目录内，不得与其他 episode 混放：

```text
recordings/<run-name>/
├── objnav.jsonl
├── metrics.json
├── visibility.json
├── planner-path.json
├── planner-path.png
├── demo.mp4
├── demo-preview.jpg
├── debug.log
└── evidence/
```

- `objnav.jsonl`：按时间排列的目标导航事件。
- `metrics.json`：从事件日志计算出的阶段延迟和 episode 汇总。
- `visibility.json`：固定出生点、目标真值以及初始视野检查。
- `planner-path.json`：planner 实际生成的路径点。
- `planner-path.png`：路径可视化。
- `demo.mp4` 和 `demo-preview.jpg`：本 episode 的录屏和预览图。
- `debug.log`：独立的原始 ROS 输出，不得混入目标导航事件日志。
- `evidence/`：目标检测帧、mask、关键截图等证据。

如果 planner 没有生成路径，仍然必须生成 `planner-path.json`，写明 `path_generated: false` 和原因；不得伪造 `planner-path.png`。如果录屏失败，必须在 `metrics.json` 写明 `demo_recorded: false` 和原因，不能放置空视频冒充成功。

## 通用字段

`objnav.jsonl` 每行是一个完整 JSON 对象。所有事件都必须包含：

- `event`：事件名称。
- `timestamp`：事件发生时的 ROS/仿真时间。
- `elapsed_s`：相对 episode 启动时间。
- `frame_id`：位姿坐标系，统一使用 `map`。

所有 timestamp 必须使用同一个 ROS clock。涉及图像的事件还必须保留原始 `frame_timestamp`，不能用回调处理时间代替图像采集时间。

为排查真实耗时，所有涉及排队、推理、通信或规划的事件还必须记录同一台机器上的 `steady_time_ns`。ROS/仿真时间用于对齐传感器和 pose，steady clock 用于计算耗时，不能使用可能因仿真暂停或跳变而失真的 ROS 时间计算 API 或处理延迟。

位姿统一写成：

```json
{
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.0,
  "qw": 1.0
}
```

每条事件必须包含以下关联字段；当前阶段尚未产生的 ID 写 `null`：

- `trace_id`：同一目标闭环全程不变。
- `image_sequence_id`：对应图像序号；不涉及图像时为 `null`。
- `track_id`
- `object_id`
- `request_id`
- `goal_id`
- `path_id`

同一次闭环中的图像、YOLO track、长期记忆节点、VLM 请求、planner goal 和路径必须能够通过这些字段互相对应。

关键处理事件必须包含 `timing`，按实际阶段填写：

```json
{
  "source_timestamp": 0.0,
  "queued_timestamp": 0.0,
  "started_timestamp": 0.0,
  "finished_timestamp": 0.0,
  "published_timestamp": 0.0,
  "downstream_received_timestamp": 0.0,
  "queue_wait_s": 0.0,
  "processing_s": 0.0,
  "transport_s": 0.0,
  "end_to_end_s": 0.0
}
```

没有对应动作的字段写 `null`，不得用同一个时间填满所有字段。发送端记录 `published_timestamp`，接收端必须在订阅回调内部记录 `downstream_received_timestamp`；没有收到时不得推测。

## 必须记录的事件

### 1. `episode_start`

记录 episode 的起点和任务：

- `run_name`
- `instruction`：原始用户指令
- `target_object`：解析后的目标类别
- `start_pose`：episode 开始时机器人 pose
- `configured_goal_pose`：如果测试预先指定终点则填写，否则为 `null`

### 2. `yolo_target_detected`

只记录 YOLO 第一次识别到目标物体的事件，不记录无关类别：

- `target_object`
- `track_id`
- `confidence`
- `bbox`
- `frame_timestamp`：被检测图像的采集时间
- `detected_timestamp`：YOLO 完成检测的时间
- `robot_pose`：该图像帧对应的机器人 pose
- `image_id` 或 `image_path`

`timestamp` 必须等于 `detected_timestamp`。机器人 pose 必须与 `frame_timestamp` 对齐，不能直接使用记录日志时的最新 odometry。

### 3. `semantic_projection_ready`

记录目标检测完成 SAM2 mask 和深度/点云投影的时间：

- `target_object`
- `track_id`
- `object_id`：此时已经关联到长期记忆节点则填写，否则为 `null`
- `source_frame_timestamp`：对应的 YOLO 图像帧时间
- `mask_completed_timestamp`：SAM2 mask 完成时间
- `projection_completed_timestamp`：深度/点云投影完成时间
- `mask_path` 或可定位 mask 的 `mask_id`
- `mask_pixel_count`
- `input_point_count`
- `point_count`：投影后有效目标点数量
- `centroid`：投影后目标在 `map` 中的三维中心
- `bbox3d`：可用时记录三维包围盒
- `projection_valid`
- `image_timestamp`
- `cloud_timestamp`
- `odom_timestamp`
- `image_cloud_delta_s`
- `image_odom_delta_s`
- `sync_wait_s`

事件的 `timestamp` 等于 `projection_completed_timestamp`。

### 4. `object_memory_associated`

记录投影结果如何进入长期物体记忆。这是判断“YOLO 已看到但为什么没有进入 VLM”的必要节点：

- `track_id`
- `object_id`
- `association_timestamp`
- `association_action`：`created`、`updated_by_track_id`、`merged_by_geometry` 或 `rejected`
- `dominant_label`
- `target_candidate`
- `association_distance_m`
- `association_iou`
- `merged_object_ids`
- `rejection_reason`

创建、更新和合并必须明确区分。被过滤或三维投影无效时也要记录 `rejected` 及真实原因。

### 5. `vlm_candidate_selected` / `vlm_candidate_rejected`

每个匹配目标类别的长期记忆节点都必须记录一次候选决策，不能只记录最终进入 VLM 的候选：

- `track_id`
- `object_id`
- `dominant_label`
- `target_object`
- `decision_timestamp`
- `selected`
- `reason`
- `is_asked_vlm`
- `queue_depth`
- 实际参与筛选的阈值及对应数值

通过筛选使用 `vlm_candidate_selected`；被过滤使用 `vlm_candidate_rejected`。不得只写笼统的 `not_selected`，必须指出具体条件。

### 6. `vlm_request_started`

VLM worker 从队列取出请求并开始准备 API 输入时记录：

- `request_id`
- `object_id`
- `dequeued_timestamp`
- `encoding_started_timestamp`
- `encoding_finished_timestamp`
- `queue_depth`
- `queue_wait_s`
- `encoding_latency_s`
- `image_size_bytes`

### 7. `vlm_submitted`

每次向 VLM 提交目标候选都必须记录：

- `request_id`
- `target_object`
- `track_id`
- `object_id`
- `submitted_timestamp`
- `input.prompt`：实际发送的完整文本 prompt
- `input.candidate_labels`
- `input.room_context`
- `input.image_id` 或 `input.image_path`
- 其他实际发送给 API 的非凭据字段

`timestamp` 等于 `submitted_timestamp`。不得记录 API key、token 或其他凭据。

### 8. `vlm_result`

每次 VLM 返回都必须记录，并通过 `request_id` 与输入一一对应：

- `request_id`
- `track_id`
- `object_id`
- `received_timestamp`
- `output.raw_output`：API 返回的完整可见文本
- `output.final_label`：程序解析后的类别
- `output.is_target`
- `output.accepted`
- `output.reason`：可用时记录模型或程序给出的理由
- `latency_s`：从提交到收到结果的时间
- `queue_wait_s`
- `api_latency_s`
- `parse_latency_s`
- `http_status`
- `retry_count`
- `timeout`
- `rate_limited`
- `model`
- `image_size_bytes`

`timestamp` 等于 `received_timestamp`。VLM 超时或请求失败也使用该事件，填写 `accepted: false`、`error` 和实际可见的错误信息。

### 9. `planner_input_sent`

记录上游实际送给 planner 的完整目标输入：

- `goal_id`
- `request_id`
- `track_id`
- `object_id`
- `target_object`
- `input.robot_pose`
- `input.target_pose`
- `input.target_bbox3d`
- `input.target_point_cloud_id`：可用时填写
- `sent_timestamp`

`timestamp` 等于 `sent_timestamp`。

### 10. `planner_input_received`

必须在 planner 的订阅回调内部记录，不能用发送端时间推测：

- `goal_id`
- `object_id`
- `received_timestamp`
- `accepted`：planner 是否接受该输入
- `reject_reason`：拒绝时必须填写
- `planner_start_pose`
- `target_pose`
- `input_latency_s`：从 `planner_input_sent` 到 planner 回调收到输入的时间

`timestamp` 等于 `received_timestamp`。

### 11. `planner_path_generated`

planner 每次生成或显著重规划目标路径时记录：

- `goal_id`
- `object_id`
- `path_id`
- `generated_timestamp`
- `start_pose`
- `target_pose`
- `path`：按执行顺序排列的 pose/waypoint 数组
- `path_length_m`
- `visualization_path`：对应本 episode 目录中 `planner-path.png` 的路径

`timestamp` 等于 `generated_timestamp`。完整路径同时写入本 episode 目录中的 `planner-path.json`。

### 12. `controller_path_received`

控制器实际收到 planner 路径时记录：

- `goal_id`
- `path_id`
- `path_received_timestamp`
- `accepted`
- `reject_reason`
- `path_point_count`
- `planner_to_controller_latency_s`

该事件必须从控制器订阅回调内部产生，不能根据 planner 发布成功推测。

### 13. `motion_started`

路径生成后第一次下发有效运动指令，以及机器人第一次产生有效位移时记录：

- `goal_id`
- `path_id`
- `first_cmd_timestamp`
- `first_motion_timestamp`
- `first_cmd_vel`
- `robot_pose`
- `planner_to_cmd_latency_s`
- `cmd_to_motion_latency_s`

有效运动阈值必须在日志中写明，不能把传感器噪声当成开始运动。

### 14. `navigation_stalled`

只在超过配置的无进展阈值时记录一次，状态恢复后允许再次记录；禁止写成周期心跳：

- `goal_id`
- `path_id`
- `robot_pose`
- `commanded_velocity`
- `actual_velocity`
- `no_progress_duration_s`
- `remaining_path_m`
- `stall_threshold_s`
- `reason`：例如 `no_cmd`、`controller_rejected_path`、`local_planner_no_valid_trajectory`、`vehicle_not_responding`

### 15. `episode_end`

无论成功、超时、主动停止或进程崩溃，都必须写最后一条事件：

- `goal_pose`：最终采用的终点 pose；从未产生目标则为 `null`
- `final_pose`：episode 结束时机器人 pose
- `goal_reached`：是否满足系统的到达阈值
- `distance_to_goal_m`：最终位置与 goal 的距离
- `active_stop`：是否由明确的 stop 指令结束
- `stop_source`：`planner`、`controller`、`user`、`test_harness`、`timeout` 或 `process_exit`
- `stop_reason`：`goal_reached`、`manual_stop`、`timeout`、`planner_failure`、`crash` 等明确原因
- `success`：仅当目标确认、planner 接管且满足到达阈值时为 `true`
- `last_completed_stage`：最后完成的阶段
- `missing_stages`：未发生的后续阶段列表

`active_stop` 和 `goal_reached` 必须分开记录。例如 planner 到达目标后主动下发停车时，两者都为 `true`；测试超时后强制停车时，`active_stop` 为 `true`、`goal_reached` 为 `false`。

## Planner 路径可视化要求

`planner-path.png` 必须基于本 episode 中 planner 实际输出的路径点生成，至少包含：

- 可用的二维地图、占据栅格或已探索区域作为背景。
- episode 起点、YOLO 首次检测位置、planner 接管位置、目标位置和最终位置。
- planner 路径折线及行进方向。
- 障碍物/不可通行区域。
- 图例、坐标轴、米制比例和 `path_id`。
- 若发生重规划，用不同颜色区分各条路径，并明确最终执行路径。

禁止用机器人 odometry 轨迹冒充 planner 规划路径。可以额外叠加实际行驶轨迹进行对比，但必须使用不同线型，并在图例中标明 `planned path` 和 `executed trajectory`。

## 完整性规则

- 事件必须按发生时间排列，不得补写虚假时间或虚假事件。
- 同一目标多次 VLM 复核、planner 拒绝或重规划必须分别记录，使用 ID 关联。
- 某阶段未发生时，不生成该阶段的伪事件；必须在 `episode_end.missing_stages` 中列出。
- 每个事件必须带 `status`：`queued`、`started`、`completed`、`rejected`、`timeout` 或 `failed`，失败时必须带 `reason` 和可见的 `error`。
- 资源数据如需采集，只能作为关键事件发生时的 `resource_snapshot`，可包含 CPU、RAM、GPU utilization 和 VRAM；禁止另写高频资源心跳污染事件日志。
- `objnav.jsonl` 只允许包含以上十五个阶段中的十六种事件（候选通过与拒绝是两种事件），不得包含 ROS 周期输出、无关物体检测、建图调试、性能心跳或 traceback。
- 原始调试输出只允许进入同一 episode 目录的 `debug.log`。
- 日志必须能直接回答：从哪里出发、何时何地看见目标、何时完成 mask 和三维投影、track 如何关联成 memory object、候选为何进入或未进入 VLM、VLM 的实际输入输出、planner 何时收到什么输入、生成了哪条路径、控制器是否开始执行、车辆是否卡住、最终停在哪里、为什么停止、是否成功到达。
