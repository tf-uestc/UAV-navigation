# Task 623: APPROACHING 阶段 VLM 闭环修正方案

## 1. 背景

当前系统在 `APPROACHING` 阶段的行为是：

```text
每隔 approach_vlm_interval 触发一次 VLM
无人机继续沿旧轨迹飞行
VLM 结果返回后触发重新规划
```

这个设计存在几个问题：

1. VLM 请求发送后，无人机不会暂停，仍然沿旧轨迹继续飞。
2. `approach_vlm_interval` 同时被当成“重检测间隔”和“VLM 超时时间”，语义混乱。
3. 当前配置中 `approach_vlm_interval = 2.0`，但 Qwen-VL API timeout 是 90 秒，mission 侧会过早认为 VLM miss。
4. `detector_node` 每收到一次 instruction 就新开线程，可能堆积 API 请求。
5. `/uav/target_world` 直接驱动 `planner_node`，旧检测结果晚到时可能污染规划。
6. `planner_node` 正在规划时如果收到新目标，会直接跳过，可能丢掉更准确的 VLM 修正目标。

目标是在不破坏现有系统功能的前提下，小范围调整 `APPROACHING` 阶段，使其变成：

```text
飞行一段时间
暂停并悬停
发送一次 VLM 请求
等待结果或 timeout
收到结果后重新规划并继续飞
结果返回或 timeout 后重新计时
```

## 2. 总体设计

保留现有主状态：

```text
IDLE
SEARCHING
APPROACHING
HOVER
```

不新增顶层 mission 状态，降低对现有逻辑的影响。只在 `APPROACHING` 内部增加子状态：

```text
APPROACH_FLYING
APPROACH_WAIT_VLM
APPROACH_WAIT_PLAN
```

推荐时序：

```text
APPROACH_FLYING
  正常跟随 trajectory_follower 输出的轨迹飞行
  从上一次 VLM 有效结果/timeout 后开始计时
  满足 approach_vlm_interval 后暂停

暂停动作
  清空 /uav/trajectory
  发布当前位置 /uav/goal_pose
  发送一次 VLM 请求
  进入 APPROACH_WAIT_VLM

APPROACH_WAIT_VLM
  持续发布当前位置 /uav/goal_pose，保持悬停
  等待 VLM 结果
  若收到有效结果，发布验证后的 /uav/approach_target
  若 timeout，增加 miss_count

APPROACH_WAIT_PLAN
  持续发布当前位置 /uav/goal_pose，保持悬停
  等待本次目标对应的新 /uav/trajectory
  收到新非空轨迹后回到 APPROACH_FLYING
  planner FAIL 后回到 SEARCHING
```

## 3. Topic 关系调整

### 3.1 当前关系

```text
detector_node
  -> /uav/target_world
  -> planner_node
```

问题：任何 `/uav/target_world` 都会直接触发规划，mission 没有机会过滤旧结果。

### 3.2 建议关系

```text
detector_node
  -> /uav/target_world
  -> mission_manager
  -> /uav/approach_target
  -> planner_node
```

`/uav/target_world` 保留，作为 detector 原始检测输出。

新增 `/uav/approach_target`，只由 `mission_manager` 发布，表示：

1. 当前 mission 状态允许使用该目标。
2. request_id 与当前等待的 VLM 请求匹配。
3. 该目标不是旧请求晚到的 stale result。

`planner_node` 增加可配置目标 topic：

```python
target_topic = rospy.get_param("~target_topic", "/uav/target_world")
rospy.Subscriber(target_topic, PointStamped, self._on_target, queue_size=1)
```

这样默认仍兼容旧链路。只在完整导航 launch 中设置：

```xml
<param name="target_topic" value="/uav/approach_target"/>
```

## 4. mission_manager 修改方案

文件：

```text
ws/src/uav_vln_bringup/scripts/mission_manager.py
```

### 4.1 新增参数

在 `mission_params.yaml` 增加：

```yaml
approach_vlm_interval: 2.0
approach_vlm_timeout: 20.0
pause_for_vlm_in_approach: true
```

含义：

| 参数 | 含义 |
|---|---|
| `approach_vlm_interval` | 上一次有效 VLM 结果或 timeout 后，至少间隔多久再重检测 |
| `approach_vlm_timeout` | VLM 请求最长等待时间 |
| `pause_for_vlm_in_approach` | approach 阶段触发 VLM 时是否暂停悬停 |

注意：`approach_vlm_interval` 不再作为 VLM timeout 使用。

还需要同步约束 detector 侧的 API timeout：

```xml
<param name="detector_args" value="{timeout: 20.0}"/>
```

或者直接增加 detector 参数，使 Qwen-VL 请求 timeout 不大于 `approach_vlm_timeout`。否则 mission 20 秒放弃等待后，detector 线程仍可能继续跑到默认 90 秒，导致后续请求 BUSY 或旧结果晚到。

### 4.2 新增内部状态

新增常量：

```python
APPROACH_FLYING = "FLYING"
APPROACH_WAIT_VLM = "WAIT_VLM"
APPROACH_WAIT_PLAN = "WAIT_PLAN"
```

新增成员变量：

```python
self._approach_phase = APPROACH_FLYING
self._approach_vlm_timeout = rospy.get_param("~approach_vlm_timeout", 20.0)
self._pause_for_vlm = rospy.get_param("~pause_for_vlm_in_approach", True)

self._pending_detection_request_id = None
self._next_request_id = 1

self._last_approach_result_time = rospy.Time(0)
self._vlm_wait_start_time = rospy.Time(0)

self._approach_hold_pose = None
self._last_valid_target = None
self._waiting_plan_since = rospy.Time(0)
self._trajectory_after_target = False
```

新增 publisher：

```python
self._approach_target_pub = rospy.Publisher(
    "/uav/approach_target", PointStamped, queue_size=1, latch=True
)
```

### 4.3 进入 APPROACHING

当前逻辑：

```python
self._last_approach_trigger_time = rospy.Time(0)
```

建议改为：

```python
self._approach_phase = APPROACH_WAIT_PLAN
self._last_approach_result_time = rospy.Time.now()
self._detection_in_flight = False
self._pending_detection_request_id = None
self._trajectory_after_target = False
self._waiting_plan_since = rospy.Time.now()

if self._last_target is not None:
    self._publish_validated_target(self._last_target)
```

原因：

1. SEARCHING 中刚收到 target 后已经有一个有效目标。
2. 进入 APPROACHING 时应先让 planner 根据该目标规划。
3. 不应立即再次触发 VLM，否则可能出现刚发现目标就停住重复请求。

### 4.4 APPPROACHING housekeeping

`_on_housekeeping_tick()` 中，`APPROACHING` 分支改为按子状态处理。

伪代码：

```python
if self._state == STATE_APPROACHING:
    if self._approach_phase == APPROACH_FLYING:
        self._handle_approach_flying(now)
    elif self._approach_phase == APPROACH_WAIT_VLM:
        self._handle_approach_wait_vlm(now)
    elif self._approach_phase == APPROACH_WAIT_PLAN:
        self._handle_approach_wait_plan(now)

    if self._check_approach_arrival():
        self._set_state(STATE_HOVER)
```

`APPROACH_FLYING`：

```python
dt = (now - self._last_approach_result_time).to_sec()
if dt >= self._approach_vlm_interval:
    if self._pause_for_vlm:
        self._pause_approach_for_vlm()
    self._trigger_approach_vlm()
    self._approach_phase = APPROACH_WAIT_VLM
    self._vlm_wait_start_time = now
```

`APPROACH_WAIT_VLM`：

```python
self._publish_hold_goal()

wait_s = (now - self._vlm_wait_start_time).to_sec()
if wait_s >= self._approach_vlm_timeout:
    self._handle_approach_vlm_timeout()
```

`APPROACH_WAIT_PLAN`：

```python
self._publish_hold_goal()
```

说明：

1. `WAIT_PLAN` 中必须持续发布 hold goal，不能可选。
2. 因为清空 `/uav/trajectory` 后，`trajectory_follower` 不再发布 `/uav/goal_pose`。
3. 如果 mission 不持续发布 hold goal，`setpoint_bridge` 会因为 `goal_timeout` 进入 `HOVER`；当前 bridge 的 `HOVER` 状态不会因为新 `/uav/goal_pose` 自动恢复 `FLYING`。
4. 不建议仅凭 `/uav/planner_status == OK` 恢复飞行，因为该状态可能是旧目标或 latched 旧状态。
5. 更稳的恢复条件是：mission 发布本次 `/uav/approach_target` 后，收到新的非空 `/uav/trajectory`。

### 4.5 暂停 approach

新增函数：

```python
def _pause_approach_for_vlm(self):
    self._clear_trajectory()
    self._capture_current_hold_pose()
    self._publish_hold_goal()
```

不要给 `setpoint_bridge` 发送 `HOVER` 命令。

低风险方式是：

1. 清空 `/uav/trajectory`，使 `trajectory_follower` 停止旧轨迹输出。
2. mission 自己持续发布当前位置 `/uav/goal_pose`。
3. `setpoint_bridge` 保持 `FLYING` 状态，只是目标点变成当前位置。

这样不需要改 `setpoint_bridge.py`。

### 4.6 触发 approach VLM

建议单独拆出：

```python
def _trigger_approach_vlm(self):
    request_id = self._next_request_id
    self._next_request_id += 1
    self._pending_detection_request_id = request_id

    data = {
        "target": {"text": self._target_text},
        MISSION_TRIGGER_KEY: True,
        "request_id": request_id,
    }
    self._instruction_pub.publish(String(data=json.dumps(data)))

    self._detection_in_flight = True
    self._last_instruction_time = rospy.Time.now()
```

`_trigger_vlm()` 不建议继续混用 SEARCHING 和 APPROACHING，因为两者 timeout 和控制语义不同。

如果短期不想拆函数，也至少需要：

```python
if self._state == STATE_APPROACHING:
    timeout = self._approach_vlm_timeout
else:
    timeout = self._search_vlm_interval * 0.9
```

### 4.7 处理 target_world

当前 `APPROACHING` 中收到 `/uav/target_world` 只是记录：

```python
rospy.loginfo("[mission] Target refined during approach")
```

建议改为：

```python
elif self._state == STATE_APPROACHING:
    if not self._is_expected_detection(msg):
        rospy.logwarn("[mission] Ignore stale detection")
        return

    self._detection_in_flight = False
    self._pending_detection_request_id = None
    self._miss_count = 0
    self._last_target = msg
    self._last_valid_target = msg
    self._last_approach_result_time = rospy.Time.now()

    self._publish_validated_target(msg)
    self._approach_phase = APPROACH_WAIT_PLAN
```

`_publish_validated_target()`：

```python
def _publish_validated_target(self, msg):
    self._approach_target_pub.publish(msg)
```

`_is_expected_detection()`：

```python
def _is_expected_detection(self, msg):
    if self._state == STATE_SEARCHING:
        return True
    if self._state != STATE_APPROACHING:
        return False
    if self._approach_phase != APPROACH_WAIT_VLM:
        return False
    if self._pending_detection_request_id is None:
        return False
    return msg.header.seq == self._pending_detection_request_id
```

注意：

1. 不能在 `APPROACHING` 中因为 `_pending_detection_request_id is None` 就返回 `True`。
2. timeout 后会清空 pending id，旧 VLM 结果可能晚到；如果此时直接返回 `True`，就会重新引入 stale result 污染规划的问题。
3. 只有 `SEARCHING -> APPROACHING` 的初始目标可以不要求 request_id，因为那个目标本身是触发进入 approach 的依据。
4. `APPROACHING` 内部的重检测结果必须满足：当前 phase 是 `APPROACH_WAIT_VLM`，且 request_id 匹配。

如果 `request_id` 未实现，第一阶段可以先按时间窗口过滤：

```python
msg.header.stamp >= self._last_instruction_time
```

但这只是过渡方案，request_id 更稳。

### 4.8 新轨迹确认与 planner 状态处理

不建议只用 `planner_status == OK` 作为恢复飞行条件。

原因：

1. `/uav/planner_status` 是普通状态 topic，可能存在旧的 `OK`。
2. 即使收到 `OK`，也不能证明它对应当前刚发布的 `/uav/approach_target`。
3. `EgoPlannerAdapter` 当前等待的是任意 `/planning/bspline`，也没有携带目标 id。

更低风险的做法是让 `mission_manager` 订阅 `/uav/trajectory`：

```python
rospy.Subscriber("/uav/trajectory", PathMsg, self._on_trajectory, queue_size=1)
```

新增变量：

```python
self._waiting_plan_since = rospy.Time(0)
self._trajectory_after_target = False
```

每次发布验证目标时：

```python
def _publish_validated_target(self, msg):
    self._trajectory_after_target = False
    self._waiting_plan_since = rospy.Time.now()
    self._approach_target_pub.publish(msg)
```

收到轨迹时：

```python
def _on_trajectory(self, msg):
    if self._state != STATE_APPROACHING:
        return
    if self._approach_phase != APPROACH_WAIT_PLAN:
        return
    if len(msg.poses) == 0:
        return
    # 确认该轨迹是在本次 approach_target 发布之后出现的。
    # Path.header.stamp 当前可能未填，第一阶段可用接收时间判断。
    if rospy.Time.now() < self._waiting_plan_since:
        return

    self._trajectory_after_target = True
    self._approach_phase = APPROACH_FLYING
    self._last_approach_result_time = rospy.Time.now()
```

`_on_planner_status()` 仍然处理失败：

```python
if status == "FAIL" and self._state == STATE_APPROACHING:
    self._set_state(STATE_SEARCHING)
```

但 `OK` 只作为日志或辅助状态，不作为离开 `WAIT_PLAN` 的唯一依据：

```python
elif status == "OK":
    rospy.loginfo_throttle(5.0, "[mission] Planner status: OK")
```

### 4.9 VLM timeout 处理

新增：

```python
def _handle_approach_vlm_timeout(self):
    self._detection_in_flight = False
    self._pending_detection_request_id = None
    self._miss_count += 1
    self._last_approach_result_time = rospy.Time.now()

    if self._miss_count >= self._max_miss:
        self._set_state(STATE_SEARCHING)
        return

    if self._last_valid_target is not None:
        self._publish_validated_target(self._last_valid_target)
        self._approach_phase = APPROACH_WAIT_PLAN
    else:
        self._set_state(STATE_SEARCHING)
```

这样一次 VLM 失败不会让系统崩溃，也不会不断发送新请求。

注意：timeout 后必须拒绝旧 request 的晚到结果。实现上依赖 4.7 的严格 `_is_expected_detection()`：`APPROACHING` 内只有 `APPROACH_WAIT_VLM` 且 request_id 匹配时才接受检测结果。

## 5. detector_node 修改方案

文件：

```text
ws/src/uav_vln_bringup/scripts/detector_node.py
```

### 5.1 解析 request_id

在 `_cb_instruction()` 中解析：

```python
request_id = data.get("request_id", 0)
```

线程参数变为：

```python
target=self._run_detection, args=(target_text, request_id)
```

### 5.2 增加 in-flight guard

新增成员：

```python
self._detecting = False
self._active_request_id = None
self._detect_lock = threading.Lock()
```

`_cb_instruction()` 中：

```python
with self._detect_lock:
    if self._detecting:
        rospy.logwarn("[detector] Detection already running, ignore request")
        self._publish_detector_status(request_id, "BUSY")
        return
    self._detecting = True
    self._active_request_id = request_id
```

`_run_detection()` 用 `try/finally` 确保释放：

```python
try:
    ...
finally:
    with self._detect_lock:
        self._detecting = False
        self._active_request_id = None
```

注意：不要静默 ignore busy 请求。至少要发布 `BUSY` 状态，让 mission 知道本次 request 没有真正开始。

### 5.3 echo request_id

发布 `/uav/target_world` 时：

```python
ps.header.seq = request_id
```

这样 mission 可以识别当前结果是否属于当前等待的请求。

### 5.4 detector_status

建议新增：

```text
/uav/detector_status std_msgs/String
```

状态内容：

```json
{"request_id": 12, "status": "STARTED"}
{"request_id": 12, "status": "SUCCEEDED"}
{"request_id": 12, "status": "FAILED"}
{"request_id": 12, "status": "BUSY"}
```

在本方案中，`detector_status` 不应只当作调试项。只要 detector 有 in-flight guard，mission 就需要知道请求是 `STARTED`、`FAILED` 还是 `BUSY`，否则会在 `WAIT_VLM` 中等待一个实际上没有开始的请求。

对本方案来说，至少需要 mission 能处理：

| 状态 | mission 行为 |
|---|---|
| `STARTED` | 保持 `APPROACH_WAIT_VLM` |
| `SUCCEEDED` | 等待 `/uav/target_world`，或仅作为日志 |
| `FAILED` | 调用 `_handle_approach_vlm_timeout()` 等价逻辑，但不必等到 timeout 秒数耗尽 |
| `BUSY` | 当前请求没有开始，应取消 pending request，短暂保持 hover 后重新计时或重试 |

如果 detector API timeout 设置为 20 秒，mission 的 `approach_vlm_timeout` 也应略大或相等，例如 20 到 25 秒。不要再出现 mission 20 秒放弃、detector 90 秒后才返回的组合。

## 6. planner_node 修改方案

文件：

```text
ws/src/uav_vln_bringup/scripts/planner_node.py
```

### 6.1 target topic 参数化

当前固定订阅：

```python
rospy.Subscriber("/uav/target_world", PointStamped, self._on_target, queue_size=1)
```

改为：

```python
target_topic = rospy.get_param("~target_topic", "/uav/target_world")
rospy.Subscriber(target_topic, PointStamped, self._on_target, queue_size=1)
```

默认值保持 `/uav/target_world`，旧 launch 不受影响。

完整链路 launch 中使用：

```xml
<param name="target_topic" value="/uav/approach_target"/>
```

### 6.2 避免丢弃最新目标

当前逻辑：

```python
if self._planning:
    rospy.loginfo("[planner] Already planning — skipping duplicate target")
    return
```

建议改为：

```python
if self._planning:
    self._pending_target = msg
    rospy.loginfo("[planner] Already planning — queued latest target")
    return
```

新增成员：

```python
self._pending_target = None
```

规划线程结束后：

```python
finally:
    self._planning = False
    if self._pending_target is not None:
        next_msg = self._pending_target
        self._pending_target = None
        self._start_planning(next_msg)
```

为了代码清晰，可以把启动线程逻辑抽成：

```python
def _start_planning(self, msg):
    self._planning = True
    thread = threading.Thread(target=self._run_planning, args=(msg,), daemon=True)
    thread.start()
```

## 7. launch 修改方案

文件：

```text
ws/src/uav_vln_bringup/launch/vlm_navigation_full.launch
```

`planner_node` 增加参数：

```xml
<node pkg="uav_vln_bringup" type="planner_node.py" name="planner_node"
      output="screen">
  <param name="planner_class"
         value="planning.ego_adapter.EgoPlannerAdapter"/>
  <param name="target_topic" value="/uav/approach_target"/>
</node>
```

`mission_manager` 保持加载 `mission_params.yaml`。

其他 launch 暂不修改，降低影响。

## 8. 不建议修改的部分

### 8.1 不建议改 setpoint_bridge

`setpoint_bridge.py` 当前负责 PX4 OFFBOARD、arm、land、setpoint 连续发布。该节点是飞控安全边界，不建议为这次闭环逻辑改它。

暂停方案应通过：

```text
清空 /uav/trajectory
mission_manager 发布当前位置 /uav/goal_pose
```

实现，而不是切换 bridge 状态。

### 8.2 不建议第一阶段新增自定义 msg

长期看，`request_id` 最好放在自定义消息中。

但第一阶段为了降低风险，可以用 `Header.seq` 携带 request_id。

后续再考虑新增：

```text
uav_vln_bringup/DetectionResult
  Header header
  uint32 request_id
  geometry_msgs/PointStamped target
  string target_text
  bool success
```

## 9. 风险控制

### 9.1 向后兼容

1. `/uav/target_world` 保留。
2. `planner_node` 默认仍订阅 `/uav/target_world`。
3. 只有 `vlm_navigation_full.launch` 切换到 `/uav/approach_target`。
4. `setpoint_bridge` 不改。
5. `trajectory_follower` 不改。

### 9.2 失败降级

1. VLM 单次 timeout 不立即回 SEARCHING。
2. 有最后有效目标时，用最后目标继续规划。
3. 连续 `max_miss` 后才回 SEARCHING。
4. planner FAIL 保持现有行为，回 SEARCHING。

### 9.3 防止资源堆积

1. mission 在 `WAIT_VLM` 中不重复发送新 VLM 请求。
2. detector 有 `_detecting` guard。
3. detector busy 时必须发布 `/uav/detector_status`，mission 不能静默等待未开始的请求。
4. detector API timeout 必须不大于或接近 `approach_vlm_timeout`，避免 mission 放弃后 detector 仍长期运行。
5. timeout 后 request_id 失效，旧结果晚到会被 mission 忽略。

## 10. 验证计划

### 10.1 静态验证

运行：

```bash
python3 -m py_compile \
  ws/src/uav_vln_bringup/scripts/mission_manager.py \
  ws/src/uav_vln_bringup/scripts/detector_node.py \
  ws/src/uav_vln_bringup/scripts/planner_node.py
```

### 10.2 单元/轻量验证

更新并运行现有：

```bash
python3 test_mission_manager.py
```

建议新增测试点：

1. mission 初始化为 `IDLE`。
2. 进入 `APPROACHING` 后先发布 `/uav/approach_target`。
3. `APPROACH_FLYING` 到达 interval 后进入 `WAIT_VLM`。
4. `WAIT_VLM` 中持续发布 hold goal。
5. 收到匹配 request_id 的 target 后进入 `WAIT_PLAN`。
6. 收到不匹配 request_id 的 target 时忽略。
7. timeout 后 miss_count 增加。
8. timeout 后旧 request_id 结果晚到时仍被忽略。
9. `WAIT_PLAN` 中持续发布 hold goal，不依赖 `trajectory_follower`。
10. 旧的 `planner_status == OK` 不会让 mission 离开 `WAIT_PLAN`。
11. 只有收到本次 `/uav/approach_target` 之后的新非空 `/uav/trajectory`，才进入 `APPROACH_FLYING`。
12. detector 返回 `BUSY` 时，mission 取消 pending request 并按重试/timeout 分支处理。

### 10.3 SITL 验证

启动仿真：

```bash
WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh
```

启动完整链路：

```bash
cd /home/tf/毕业设计/ws
source /opt/ros/noetic/setup.bash
source src/ego-planner/devel/setup.bash
source devel/setup.bash
source /home/tf/毕业设计/.env
roslaunch uav_vln_bringup vlm_navigation_full.launch
```

发送命令：

```bash
rostopic pub -1 /uav/state_cmd std_msgs/String "data: 'takeoff'"
rostopic pub -1 /uav/instruction std_msgs/String "data: '{\"target\":{\"text\":\"红色支架\"}}'"
```

观察：

```bash
rostopic echo /uav/mission_status
rostopic echo /uav/planner_status
rostopic echo /uav/target_world
rostopic echo /uav/approach_target
rostopic echo /uav/detector_status
rostopic echo /uav/trajectory
rostopic hz /uav/goal_pose
```

期望：

1. 搜索阶段正常发布搜索 setpoint。
2. 找到目标后进入 `APPROACHING`。
3. `planner_node` 只响应 `/uav/approach_target`。
4. approach 中触发 VLM 时，旧 `/uav/trajectory` 被清空。
5. 等 VLM 时无人机悬停。
6. VLM 结果回来后重新规划并继续飞。
7. 旧 request_id 结果不会触发规划。
8. detector 不会并发开多个 API 请求。
9. `WAIT_PLAN` 阶段不会因为旧 `planner_status OK` 提前恢复飞行。
10. 清空轨迹后 mission 仍持续发布 hold goal，`setpoint_bridge` 不会因 goal timeout 卡入 HOVER。
11. detector API timeout 与 mission wait timeout 一致，不出现 20 秒放弃但 90 秒后才返回的旧结果。

## 11. 分阶段实施建议

### 阶段 1：低风险 topic 隔离

1. `planner_node` 增加 `~target_topic` 参数。
2. `mission_manager` 增加 `/uav/approach_target` publisher。
3. 完整 launch 中让 planner 订阅 `/uav/approach_target`。
4. mission 暂时把合法目标原样转发。

验证：完整链路仍能规划和飞行。

### 阶段 2：approach 暂停等待

1. 增加 approach 子状态。
2. 增加 hold pose 发布。
3. VLM 触发后清空轨迹并进入 `WAIT_VLM`。
4. 结果返回后再发布 `/uav/approach_target`。

验证：approach 中能停住等 VLM。

### 阶段 3：request_id 与 detector guard

1. instruction JSON 增加 request_id。
2. detector echo 到 `Header.seq`。
3. mission 过滤 stale result。
4. detector 增加 `_detecting` 防并发。
5. detector 发布 `/uav/detector_status`。
6. mission 处理 `BUSY` / `FAILED` / `SUCCEEDED`。
7. detector API timeout 配置为不大于或接近 `approach_vlm_timeout`。

验证：旧结果晚到不会触发规划，不会堆 API。

### 阶段 4：轨迹确认与 planner pending target

1. mission 订阅 `/uav/trajectory`。
2. mission 只在收到本次目标之后的新非空 trajectory 后离开 `WAIT_PLAN`。
3. planner 正在规划时缓存最新目标。
4. 当前规划结束后立即处理 pending target。

验证：旧 `planner_status OK` 不会提前恢复飞行，快速连续目标修正不会丢最新目标。

## 12. 预期最终效果

改造完成后：

1. `APPROACHING` 阶段不会在等待 VLM 时继续飞旧轨迹。
2. VLM 请求返回或 timeout 后才重新开始计时 2 秒。
3. API timeout 与重检测间隔解耦。
4. detector 不会堆积线程和 API 请求。
5. stale VLM 结果不会污染 planner。
6. planner 不会直接消费未验证的 `/uav/target_world`。
7. 最新修正目标不会因为 planner busy 被直接丢弃。
8. mission 不会因为旧 `planner_status OK` 提前恢复飞行。
9. `WAIT_PLAN` 和 `WAIT_VLM` 中持续 hold，避免 `setpoint_bridge` goal timeout 后卡入 HOVER。
10. `setpoint_bridge` 不变，飞控链路风险较低。
