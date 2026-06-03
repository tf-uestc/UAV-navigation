# Draft: 项目清理 + 新世界模型 + 去 v1

## 需求 1: .env 自动加载
- 当前: 每次启动前要手动 `source .env` 或 `export VLM_API_KEY=...`
- 目标: roslaunch 自动加载 .env

## 需求 2: 删除 dummy detector
- 目标: 清理 `dummy_detector.py`，只保留 QwenVL production 代码
- 同时更新 CMakeLists.txt / launch 默认值

## 需求 4: 新 Gazebo 世界模型
详细规格：
- SDF 1.7 格式，ODE 物理引擎
- 全域提亮光照，消除黑暗
- 地面: 深灰色哑光平面
- 无人机: 外部 spawn 注入，坐标 X:-1.01, Y:-0.98, Z:0.83
- **黄色起降平台**: 圆柱体 r=1.3m, h=0.02m, 中心(-1.01,-0.98,0.01), 哑光淡黄 RGB(1,0.92,0.15), 顶面白色十字标记
- **障碍物 A**: 位置(0.25,0.22,0.12), 低暗红圆柱 长0.9m r=0.05m, 三根黑色细支架(r=0.03m)三角分叉
- **障碍物 B**: 位置(0.42,-2.15,0.12), 同结构
- **光照**: ambient 0.65,0.65,0.65; 平行太阳光 漫反射0.92,0.92,0.92 软阴影; 点光源(0,0,9)暖白色
- 相机初始视角: 俯视斜拍，能完整包含平台+两障碍物
- 文件落地在 simulation/px4_gazebo_classic/worlds/

## 需求 5: 去掉 V1 版本
- 删除 `vlm_navigation.py` 从 CMakeLists.txt install
- 删除或标记 deprecated vlm_navigation.launch
- 更新 scripts_demo_all.sh 指向 v2

## 当前代码库状态
- `detector_node.py`: 已修复 latch + vlm_model 传参
- `qwen_vl_grounding.py`: 日志已改为 rospy
- `vlm_navigation_v2.launch`: 已修复 arg + 改为 qwen3.6-plus
- `.env`: 已在 .gitignore，未追踪
- `vlm_navigation.py`: v1 旧版，仍保留
- `vlm_navigation.launch`: v1 launch
- `dummy_detector.py`: 测试用，要删除
- `base_detector.py`: ABC，保留
