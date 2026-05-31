# F2 — Tasks

每条对应约 1 个 commit。前置：F3 已完成（或至少 TF 部分完成，否则 F2 输出不到 map 系）。

## 0. 准备

- [ ] 在 `ws/src/uav_vln_bringup/scripts/` 下新建 `detection/` 目录与 `__init__.py`
- [ ] 把 API key 从 `vlm_navigation.launch` 拿掉，改为从环境变量 `VLM_API_KEY` 读
- [ ] 在 `docs/secrets.md`（或 `.env.example`）记下需要哪些环境变量

## 1. Detector ABC

- [ ] 写 `detection/base.py`：`Detection` dataclass + `Detector` ABC（按 plan.md §2）
- [ ] 写 1 个最小 dummy 实现 `detection/dummy.py`：固定返回画面正中央 100×100 框，方便测节点骨架
- [ ] 在 `detection/__init__.py` 暴露 `Detector`、`Detection`

## 2. QwenVLGroundingDetector

- [ ] 写 `detection/qwen_vl_grounding.py`：迁移 `vlm_navigation.py:VLMApiClient` 的 HTTP 调用部分
- [ ] 实现 `_parse_bbox`：四种正则 + 归一化判定
- [ ] 实现 `depth_in_bbox`：框内中位数 + 有效率检查
- [ ] 实现 `Detector.detect()`：组合上面三步，返回 `Detection`

## 3. detector_node

- [ ] 写 `detector_node.py`：通用入口，按 `~detector_class` importlib 加载
- [ ] 订阅 `/camera/rgb/image_raw`、`/camera/depth/image_raw`、`/depth/camera_info`、`/uav/instruction`
- [ ] 用 `tf2_buffer.transform()` 把相机系点变换到 `map`（F3 接口）
- [ ] 发布 `/uav/target_world` (`PointStamped`)
- [ ] 发布 `/uav/target_debug` (`Image`)：bbox + 中心 + 文字注释
- [ ] 在 `CMakeLists.txt` 的 `catkin_install_python` 列表加上新脚本

## 4. Launch

- [ ] 复制 `vlm_navigation.launch` → `vlm_navigation_v2.launch`
- [ ] 在 v2 里换成 `detector_node`，参数按 plan.md §6
- [ ] 老 launch 保持不动作回退

## 5. 验证（无需上飞机）

- [ ] 用 dummy detector 跑通节点骨架，看 `/uav/target_debug` 出图
- [ ] 切换到 QwenVLGroundingDetector，跑一帧，**目视确认框位置正确**（重点：归一化判定）
- [ ] 用静态 rosbag 跑 30 帧，统计解析成功率
- [ ] 跑 10 次拔网线 / 限流，验证节点不崩、不阻塞

## 6. 飞行验证（依赖 F3 完成）

- [ ] 在 `empty.world` 放一个红色立方体，让无人机起飞后调用 F2
- [ ] 对比新旧链路在同一组场景下的 3D 定位 RMSE（即 spec.md §4 的指标表）
- [ ] 截一张 `/uav/target_debug` 截图存到 `docs/features/F2-grounding/screenshots/`

## 7. 论文素材沉淀

- [ ] 把 §4 的指标表写到 `docs/experiments/F2_baseline_vs_grounding.md`
- [ ] 录一段 30s 的对比视频（baseline 抖 vs grounding 稳）

## 完成标准

- spec.md §4 表里 5 个指标全部达标
- 老 launch 仍能跑（兼容性）
- detector_class 切到 dummy 也能跑（解耦验证）
