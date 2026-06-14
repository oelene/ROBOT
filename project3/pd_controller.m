function tau = pd_controller(q, qd, q_des, qd_des, qdd_ref, params, gains)
%PD_CONTROLLER 关节空间 PD 控制器（含前馈加速度）
%
%   tau = pd_controller(q, qd, q_des, qd_des, qdd_ref, params, gains)
%
%   控制律：
%       tau = Kp .* (q_des − q) + Kd .* (qd_des − qd) + qdd_ref
%
%   输入：
%       q       : 1×n 当前关节位置
%       qd      : 1×n 当前关节速度
%       q_des   : 1×n 期望关节位置
%       qd_des  : 1×n 期望关节速度
%       qdd_ref : 1×n 前馈加速度（可为零向量）
%       params  : 机械臂参数结构体（保留接口，本控制器未使用动力学参数）
%       gains   : struct，包含
%                   gains.Kp  1×n 比例增益
%                   gains.Kd  1×n 微分增益
%
%   输出：
%       tau     : 1×n 关节控制量（在 simulate_tracking 的简化等效惯性
%                 假设下等价于关节加速度指令）
%
%   说明：
%   本课程不要求建模完整动力学，因此 tau 在 simulate_tracking 内部
%   按"等效惯性矩阵 = I"折算为关节加速度。如需要扩展到含重力补偿的
%   计算力矩控制，可在本函数末尾追加 G(q) 项。

    % 所有状态统一成 1×n 行向量，避免输入为列向量时逐元素运算扩维。
    q       = q(:)';
    qd      = qd(:)';
    q_des   = q_des(:)';
    qd_des  = qd_des(:)';
    qdd_ref = qdd_ref(:)';

    % 当前控制器采用逐关节对角增益，每个关节独立使用自己的 Kp、Kd。
    Kp = gains.Kp(:)';
    Kd = gains.Kd(:)';

    % 比例项：位置偏差越大，拉回目标的作用越强。
    % 微分项：抑制速度偏差和振荡，相当于增加阻尼。
    % qdd_ref：期望加速度前馈，减少只靠误差追赶轨迹造成的滞后。
    % 使用 .* 是逐关节相乘，不是完整矩阵乘法。
    tau = Kp .* (q_des - q) + Kd .* (qd_des - qd) + qdd_ref;
end
