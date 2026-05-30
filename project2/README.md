# Project 2：机械臂逆运动学求解与验证

## 一、项目简介

本项目为《机器人》课程设计的第二个阶段任务，主要围绕机械臂的**逆运动学** (Inverse Kinematics, IK) 展开。项目目标是帮助你掌握从齐次变换矩阵反推关节变量的完整方法，重点掌握课堂讲授的 **解析法 (闭式解)**，并通过测试函数验证求解结果的正确性。

本项目以 Project 1 中已经完成的 MDH 建模和正运动学代码为基础。**运行前请把 Project 1 与 Project 2 的全部 `.m` 文件放在同一个文件夹下** (作为 Matlab 当前工作目录)，否则将无法调用 `robot_params`、`forward_kinematics` 等函数。

---

## 二、任务要求

延续 Project 1 中所选的机械臂型号 (CR7 或 SR3)，完成以下内容：

1. 根据课堂讲授的 **Pieper 分离法**，在课程设计报告中给出完整的解析法逆运动学推导，包括：
   - 腕中心 (wrist center) 的求解；
   - 前 3 关节 (位置子问题) 的求解，含肩部、肘部多解分析；
   - 后 3 关节 (姿态子问题) 的求解，含腕部翻转多解分析。
2. 在 Matlab 中实现解析法 IK，保留全部多解输出 (最多 8 组)。
3. 实现解的筛选与挑选 (按距离参考关节角最近，可选地按 `qlim` 过滤)。
4. 调用 RTB 工具箱的 `ikine6s` 或 `ikunc` 对自编写结果进行对比验证。
5. (备选) 实现几何雅可比与 DLS 数值 IK 作为对照。

---

## 三、代码框架说明

```
project2_inverse_kinematics/
├── main_project2_test.m              入口，串起 4 个测试
├── inverse_kinematics_analytical.m   ★ 解析法 IK (核心)
├── select_ik_solution.m              多解选择
├── jacobian_geometric.m              几何雅可比 (备选项目使用)
├── inverse_kinematics_numerical.m    DLS 数值 IK (备选)
├── test_ik_roundtrip.m               q→FK→IK→q' 自洽性测试
├── test_ik_vs_rtb.m                  与 RTB 工具箱对比
├── test_jacobian.m                   解析雅可比 vs 数值差分
└── test_ik_numerical.m               数值 IK 收敛测试 (备选)
```

依赖 (来自 Project 1)：

```
robot_params.m           forward_kinematics.m       mdh_transform.m
build_mdh_table.m        build_rtb_robot.m
```

---

## 四、需要完成的【填空】

未填的 TODO 处变量初始化为 `NaN`，运行测试时会自动报错并提示具体哪一处尚未完成。

### 1. `inverse_kinematics_analytical.m`★ (主线，必做)

| TODO | 含义 | 形式提示 |
|------|------|--------|
| TODO 1 | 腕中心 `Pw` (3×1) | `Pw = p − d6 · R(:,3)` |
| TODO 2 | `theta1` 两组解 (2×1) | `atan2(py,px)` 与 `atan2(py,px)+π` |
| TODO 3 | 平面投影 `r`、`s` | `r = px·cos(th1)+py·sin(th1)`、`s = pz − d1` |
| TODO 4 | 余弦定理求 `cos_phi` | `(r²+s²−a2²−L²)/(2·a2·L)`，其中 `L=√(a3²+d4²)` |
| TODO 5 | 求 `theta2` 的两个辅助角 `alpha`、`beta` | `atan2(s,r)` 与 `atan2(L·sin_phi, a2+L·cos_phi)` |
| TODO 6 | `R36 = R03ᵀ · R` | 一行矩阵乘法 |
| TODO 7 | 由 `R36` 求 `theta4`、`theta5`、`theta6` | ZYZ 形式的轴角提取，含奇异分支 |

### 2. `select_ik_solution.m` (开放性任务)

解析法 IK 一般返回多组解，需要根据所选机械臂的工作场景自行设计选解方案。本文件给出函数接口与默认占位 (返回第一组解)，请你：

- 设计自己的选解方案；
- 在课程设计报告中说明所依据的指标、取舍规则与设计理由；
- 在文件中标记的 `TODO` 处替换默认实现。

### 3. `jacobian_geometric.m` (建议完成，为 Project 3 做准备)

| TODO | 含义 |
|------|------|
| TODO 1 | 第 i 列线速度部分 `Jv = z_{i−1} × (p_n − p_{i−1})` |
| TODO 2 | 第 i 列角速度部分 `Jw = z_{i−1}` |

### 4. `inverse_kinematics_numerical.m` (备选)

| TODO | 含义 |
|------|------|
| TODO 1 | DLS 增量公式 `Δq = Jᵀ (J·Jᵀ + λ²·I)⁻¹ e` |
| TODO 2 | 用 `dq` 更新 `q` (注意维度匹配) |

> 模板提示中的形式仅是常见参考，具体公式应与你课程报告中的 MDH 推导保持一致。
> 尤其要注意：**MDH 偏置项符号**、**ZYZ 与 ZYX 提取索引差别**。

---

## 五、推荐完成顺序

```
第 1 步：jacobian_geometric.m  (2 处 TODO，作为热身验证 FK 调用是否正常)
        → 运行 test_jacobian，期望 [PASS]

第 2 步：inverse_kinematics_analytical.m TODO 1, 2
        (腕中心 + theta1)
        → 此时 test_ik_roundtrip 仍会因 TODO 3 未完成而 FAIL，
          但报错信息会准确定位

第 3 步：TODO 3, 4, 5  (前 3 关节)
        → 运行 test_ik_roundtrip，
          若位置误差 < 1e-4 而姿态误差 > 1e-4，说明 TODO 1-5 已对，
          只剩腕部待做

第 4 步：TODO 6, 7  (后 3 关节)
        → test_ik_roundtrip 与 test_ik_vs_rtb 全部 [PASS]

第 5 步 (备选)：inverse_kinematics_numerical.m
        → test_ik_numerical 全部 [PASS]
```

---

## 六、判分依据

每个测试函数都会逐用例打印 `[PASS]` 或 `[FAIL]`，并在末尾汇总通过用例数。

| 测试 | 通过判据 |
|------|----------|
| `test_ik_roundtrip` | 解集中每一组解满足 `FK(q')=FK(q)` (位置 / 姿态误差均 `< 1e-4`)，且解集中存在与原始 q 距离 `< 1e-3` 的解 |
| `test_ik_vs_rtb` | 解集中存在与 `ikine6s/ikunc` 距离 `< 1e-3` 的解，FK 验证误差极小 |
| `test_jacobian` | 解析 J 与数值差分 J 的 Frobenius 误差 `< 1e-4` |
| `test_ik_numerical` | DLS 收敛、最终位姿误差 `< 1e-4` |

`[FAIL]` 时，测试会自动打印诊断提示，例如：

- 位置误差大 → 重点检查 TODO 1 / 3 / 4 / 5
- 姿态误差大 → 重点检查 TODO 6 / 7
- FK 一致但解集不含原 q → 检查 TODO 2 是否漏写第二组解

---

## 七、运行方法

### 1. 环境要求

- 已安装 Matlab；
- 已安装 Peter Corke Robotics Toolbox 并加入搜索路径；
- **将 Project 1 中已经填好 TODO 的 `.m` 文件与本项目所有 `.m` 文件放到同一个文件夹下**。

### 2. 运行入口

在 Matlab 中打开项目文件夹后，直接运行：

```matlab
main_project2_test
```

如需切换机械臂型号，只需修改 `main_project2_test.m` 中的 `robot_type` 变量。

---

## 八、与 Project 1 / Project 3 的衔接

- **来自 Project 1**：复用 MDH 建模、FK 函数与 RTB 模型构建。
- **指向 Project 3**：本项目实现的 IK 与雅可比将作为 Project 3 (路径规划 / 控制) 的基础，请保持 `inverse_kinematics_analytical` 与 `jacobian_geometric` 的函数接口稳定，便于后续直接调用。
