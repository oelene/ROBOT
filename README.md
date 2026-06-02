# ROBOT 课程设计

本仓库整理了《机器人》课程设计的三个阶段任务，当前实现围绕 **xMate SR3** 机械臂展开。

## 项目结构

| 目录 | 内容 |
|---|---|
| `project1/` | SR3/CR7 机械臂参数、MDH 建模、正运动学与 RTB 模型构建 |
| `project2/` | 逆运动学、几何雅可比、IK 解选择等运动学扩展 |
| `project3/` | 路径规划、轨迹生成、PD 跟踪仿真与额外避障可视化 |

## Project 3 亮点

- 默认机械臂型号：`SR3`
- 路径规划：关节空间直线检测 + 双向 RRT + shortcut 路径简化
- 避障检测：整条机械臂连杆线段与球形障碍物距离检测
- 轨迹生成：基于路径弧长的五次多项式时间缩放
- 额外测试：5 组自定义场景，覆盖无障碍、单球挡路、三球连续挡路
- 可视化：主入口结束后会额外展示多组 SR3 绕障动画

## 快速运行

在 Matlab 中把 `project1/`、`project2/`、`project3/` 加入路径，然后进入 `project3/`：

```matlab
addpath('../project1');
addpath('../project2');
addpath('../project3');
cd('../project3');
main_project3_test
```

也可以单独运行额外测试与可视化：

```matlab
test_extra_path_cases('SR3')
visualize_extra_path_cases('SR3')
```

## 报告

Project 3 的路径规划与轨迹生成说明文档位于：

- `project3/project3_path_trajectory_report.tex`
- `project3/project3_path_trajectory_report.pdf`

## 环境要求

- Matlab
- Peter Corke Robotics Toolbox
- XeLaTeX，仅用于重新编译中文报告 PDF
