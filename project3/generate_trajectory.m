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

    if size(path, 2) ~= n || size(path, 1) < 2
        error('轨迹生成失败：path 维度应为 M×params.n，且 M >= 2。');
    end

    traj.t = linspace(0, scene.T_total, K);

    % 用五次多项式时间缩放 s(t)，沿 path 的关节空间弧长前进。
    tau = traj.t / scene.T_total;
    s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

    ds_dt   = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / scene.T_total;
    d2s_dt2 = (60*tau - 180*tau.^2 + 120*tau.^3) / (scene.T_total^2);

    seg_vec = diff(path, 1, 1);
    seg_len = sqrt(sum(seg_vec.^2, 2));
    keep = [true; seg_len > 1e-12];
    path = path(keep, :);
    seg_vec = diff(path, 1, 1);
    seg_len = sqrt(sum(seg_vec.^2, 2));

    if isempty(seg_len) || sum(seg_len) < 1e-12
        traj.q   = repmat(path(1, :), K, 1);
        traj.qd  = zeros(K, n);
        traj.qdd = zeros(K, n);
    else
        u_node = [0; cumsum(seg_len) / sum(seg_len)];
        u = s(:);

        traj.q   = zeros(K, n);
        traj.qd  = zeros(K, n);
        traj.qdd = zeros(K, n);

        for k = 1:K
            if u(k) >= 1
                idx = numel(seg_len);
                local = 1;
            else
                idx = find(u(k) >= u_node(1:end-1) & u(k) <= u_node(2:end), 1, 'first');
                if isempty(idx)
                    idx = 1;
                end
                denom = u_node(idx+1) - u_node(idx);
                local = (u(k) - u_node(idx)) / denom;
            end

            dq_du = seg_vec(idx, :) / (u_node(idx+1) - u_node(idx));
            traj.q(k, :)   = path(idx, :) + local * seg_vec(idx, :);
            traj.qd(k, :)  = dq_du * ds_dt(k);
            traj.qdd(k, :) = dq_du * d2s_dt2(k);
        end

        traj.q(1, :)     = path(1, :);
        traj.q(end, :)   = path(end, :);
        traj.qd(1, :)    = 0;
        traj.qd(end, :)  = 0;
        traj.qdd(1, :)   = 0;
        traj.qdd(end, :) = 0;
    end

    if any(~isfinite(traj.t)) || any(~isfinite(traj.q(:))) ...
       || any(~isfinite(traj.qd(:))) || any(~isfinite(traj.qdd(:)))
        error('轨迹生成失败：generate_trajectory.m 中的 TODO 尚未完成。');
    end
end
