# Project 3：路径规划、轨迹生成与跟踪仿真

## 一、项目简介

本项目为《机器人》课程设计的第三个阶段任务，主要围绕

- 任务空间避障路径规划
- 关节空间轨迹生成
- 关节空间 PD 跟踪与 RTB 动画仿真

三个环节展开。本项目以 Project 1（FK）与 Project 2（IK / 雅可比）的代码为基础。

**运行前请把 Project 1 与 Project 2 中已经填好 TODO 的 `.m` 文件，与本项目所有 `.m` 文件放在同一个文件夹下（作为 Matlab 当前工作目录）**，否则将无法调用 `robot_params`、`forward_kinematics`、`build_rtb_robot` 等函数。

---

## 二、任务要求

延续 Project 1 中所选的机械臂型号（CR7 或 SR3），完成以下内容：

1. 在所给场景（球形障碍 + 起终点关节角）下，实现一条由 `scene.q_start` 到 `scene.q_goal` 的关节空间路径，保证可达且不与任何障碍碰撞。
2. 将离散路点扩展为时间参数化的连续轨迹，保证位置 / 速度 / 加速度满足边界条件，且峰值在机械臂可行范围内。
3. 调用本项目提供的 `pd_controller.m` 与 `simulate_tracking.m`，使机械臂在 RTB 中沿轨迹运动；理解运动学动画与简化动力学仿真两种模式下的差异。
4. 在课程设计报告中说明所选算法、参数与结果。

---

## 三、代码框架说明

```
project3/
├── main_project3_test.m       入口（已写好）
├── scene_easy.m               场景：简单（已写好）
├── scene_hard.m               场景：困难（已写好）
├── plan_path.m                ★ 学生填：路径规划
├── generate_trajectory.m      ★ 学生填：轨迹生成
├── pd_controller.m            已写好：关节空间 PD 控制器
├── simulate_tracking.m        已写好：跟踪仿真主循环（含 RTB 动画）
├── test_path_planning.m       已写好：路径合法性测试
├── test_trajectory.m          已写好：轨迹平滑性测试
└── test_tracking.m            已写好：端到端跟踪测试
```

依赖（来自 Project 1 / Project 2）：

```
robot_params.m            forward_kinematics.m       mdh_transform.m
build_mdh_table.m         build_rtb_robot.m
inverse_kinematics_analytical.m   jacobian_geometric.m
select_ik_solution.m
```

---

## 四、接口契约

跨文件统一的数据结构与函数签名如下（**建议不要修改签名**，确有扩展请用可选参数或结构体字段）：

### 场景 `scene`

| 字段 | 维度 | 含义 |
|---|---|---|
| `name`       | string  | 场景名 |
| `q_start`    | 1×n     | 起始关节角（弧度） |
| `q_goal`     | 1×n     | 目标关节角（弧度） |
| `obstacles`  | M×4     | 球形障碍 `[cx, cy, cz, r]`，长度单位与 `params.a/d` 一致 |
| `T_total`    | 标量    | 推荐总运动时长（秒） |
| `dt`         | 标量    | 离散步长（秒） |

### 路径 `path`

`path = plan_path(scene, params)`

- `path` 为 `M×n` 关节空间路点矩阵；
- `path(1,:) == scene.q_start`、`path(end,:) == scene.q_goal`；
- 每个路点须在 `params.qlim` 内；
- 相邻路点连线在任务空间下不应穿过 `scene.obstacles`。

### 轨迹 `traj`

`traj = generate_trajectory(path, scene, params)`

| 字段 | 维度 | 约定 |
|---|---|---|
| `traj.t`   | 1×K | `t(1)=0`、`t(end)=scene.T_total`、等距步长 `scene.dt` |
| `traj.q`   | K×n | 关节位置；首末行 == `path` 的首末行 |
| `traj.qd`  | K×n | 关节速度；首末行 == 0 |
| `traj.qdd` | K×n | 关节加速度；首末行 == 0 |

`K = round(T_total/dt) + 1`。

### 仿真结果 `result`

`result = simulate_tracking(traj, scene, params, mode)`

`mode` 可选 `'kinematic'`（默认）或 `'dynamic'`。返回字段见 `simulate_tracking.m` 的函数头。

---

## 五、需要完成的TODO

| 文件 | TODO 数 | 简述 |
|---|---|---|
| `plan_path.m`           | 1（开放） | 任意符合接口契约的路径规划算法 |
| `generate_trajectory.m` | 4（开放） | 任意符合接口契约的时间参数化方法 |

未填的 TODO 处变量初始化为 `NaN`，运行测试时会自动报错并提示具体哪一处尚未完成。

---

## 六、判分依据

每个测试函数都会逐用例打印 `[PASS]` 或 `[FAIL]`，并在末尾汇总通过用例数。

| 测试 | 通过判据 |
|---|---|
| `test_path_planning` | 维度合法、首末点匹配、路点在 qlim 内、采样下不与障碍碰撞 |
| `test_trajectory`    | 字段维度匹配、时间序列等距、端点位置/速度/加速度满足约束、中段差分一致性 |
| `test_tracking`      | `'kinematic'` 模式下末端误差 < 1e-6；`'dynamic'` 模式下末端误差小于场景尺度的 5% |

---

## 七、运行方法

### 1. 环境要求

- 已安装 Matlab；
- 已安装 Peter Corke Robotics Toolbox 并加入搜索路径；
- 将 Project 1 / Project 2 中已经填好的 `.m` 文件与本项目所有 `.m` 文件放在同一文件夹下。

### 2. 运行入口

在 Matlab 中打开项目文件夹后，直接运行：

```matlab
main_project3_test
```

如需切换机械臂型号、场景或仿真模式，请修改 `main_project3_test.m` 顶部的：

```matlab
robot_type = 'CR7';
scene_name = 'easy';
sim_mode   = 'kinematic';
```

---

## 八、与 Project 1 / Project 2 的衔接

- **来自 Project 1**：MDH 建模、`forward_kinematics`、RTB 模型构建。
- **来自 Project 2**：`inverse_kinematics_analytical`（若在路径规划中调用任务空间→关节空间转换时使用）、`jacobian_geometric`、`select_ik_solution`。
- **本项目对外接口**：`plan_path`、`generate_trajectory` 的签名稳定，可在课程后续扩展（实物部署、视觉引导等）中直接调用。
