function traj = generate_trajectory(path, scene, params)
%GENERATE_TRAJECTORY 把离散关节路点 path 扩展成时间参数化的连续轨迹
%
%   traj = generate_trajectory(path, scene, params)
%
%   输入：
%       path   : M×n 关节路点矩阵（来自 plan_path）
%       scene  : 场景结构体，使用其 T_total 与 dt 字段
%       params : 机械臂参数结构体
%
%   输出：
%       traj   : 轨迹结构体，字段约定如下
%                  traj.t    1×K  时间序列，t(1)=0、t(end)=scene.T_total
%                  traj.q    K×n  关节位置序列，首行 == path(1,:)、末行 == path(end,:)
%                  traj.qd   K×n  关节速度序列，首末行均为 0
%                  traj.qdd  K×n  关节加速度序列，首末行均为 0
%                K = round(T_total/dt) + 1
%
%   说明：
%   本文件为开放性任务。插值方法可由小组自行选定（线性、三次多项式、
%   五次多项式、梯形速度规划、S 形规划 等），实现后请在课程设计报告中说明：
%       1. 所选插值方法；
%       2. 边界条件（首末速度 / 加速度的设置方式）；
%       3. 关节速度 / 加速度峰值与机械臂可行范围的对比。

    % =====================================================================
    % TODO: 实现你们组的轨迹生成
    % ---------------------------------------------------------------------
    %   要求：
    %     - traj.t、traj.q、traj.qd、traj.qdd 字段维度满足上述约定
    %     - traj.q 在首末点与 path 一致
    %     - traj.qd、traj.qdd 在首末时刻为 0
    %     - 离散步长 dt 取 scene.dt
    %
    %   未填时各字段为 NaN，下游测试会精确报错指向本 TODO。
    % =====================================================================
    K = round(scene.T_total / scene.dt) + 1;
    n = params.n;

    traj.t   = NaN(1, K);   % TODO
    traj.q   = NaN(K, n);   % TODO
    traj.qd  = NaN(K, n);   % TODO
    traj.qdd = NaN(K, n);   % TODO

    if any(~isfinite(traj.t)) || any(~isfinite(traj.q(:))) ...
       || any(~isfinite(traj.qd(:))) || any(~isfinite(traj.qdd(:)))
        error('轨迹生成失败：generate_trajectory.m 中的 TODO 尚未完成。');
    end
end
