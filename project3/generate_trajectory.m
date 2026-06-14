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
    % 时间区间包含 t=0 和 t=T_total 两个端点，所以采样点数要比
    % 时间小区间的数量多 1。
    K = round(scene.T_total / scene.dt) + 1;
    n = params.n;

    if size(path, 2) ~= n || size(path, 1) < 2
        error('轨迹生成失败：path 维度应为 M×params.n，且 M >= 2。');
    end

    % linspace 保证首末时间精确命中 0 和 T_total。
    traj.t = linspace(0, scene.T_total, K);

    % 用五次多项式时间缩放 s(t)，沿 path 的关节空间弧长前进。
    % tau 是归一化时间，范围从 0 变化到 1。
    tau = traj.t / scene.T_total;

    % s(tau)=10*tau^3-15*tau^4+6*tau^5 满足：
    % s(0)=0、s(1)=1，且首末一阶、二阶导数都为 0。
    % 因此机器人从静止开始，并在终点平稳停下。
    s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

    % s 对真实时间 t 的一阶、二阶导数。
    % 分母 T_total 和 T_total^2 来自 tau=t/T_total 的链式法则。
    ds_dt   = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / scene.T_total;
    d2s_dt2 = (60*tau - 180*tau.^2 + 120*tau.^3) / (scene.T_total^2);

    % seg_vec(i,:) 是第 i 段的六维关节增量 q_{i+1}-q_i。
    seg_vec = diff(path, 1, 1);

    % 使用关节空间欧氏距离作为每段的“弧长”。
    seg_len = sqrt(sum(seg_vec.^2, 2));

    % 删除与前一个路点重复的点，避免产生零长度区间和除零。
    keep = [true; seg_len > 1e-12];
    path = path(keep, :);
    seg_vec = diff(path, 1, 1);
    seg_len = sqrt(sum(seg_vec.^2, 2));

    if isempty(seg_len) || sum(seg_len) < 1e-12
        traj.q   = repmat(path(1, :), K, 1);
        traj.qd  = zeros(K, n);
        traj.qdd = zeros(K, n);
    else
        % 将累计弧长归一化到 [0,1]。较长的路径段获得较大的 u 区间，
        % 因而在统一时间缩放下也会分配到更多采样时刻。
        u_node = [0; cumsum(seg_len) / sum(seg_len)];
        u = s(:);

        traj.q   = zeros(K, n);
        traj.qd  = zeros(K, n);
        traj.qdd = zeros(K, n);

        for k = 1:K
            % 找出当前进度 u(k) 位于哪个分段 [u_node(i),u_node(i+1)]。
            if u(k) >= 1
                % 最后一个采样点强制使用最后一段末端，避免浮点越界。
                idx = numel(seg_len);
                local = 1;
            else
                idx = find(u(k) >= u_node(1:end-1) & u(k) <= u_node(2:end), 1, 'first');
                if isempty(idx)
                    idx = 1;
                end
                denom = u_node(idx+1) - u_node(idx);
                % local 是当前分段内部的线性插值比例，范围约为 [0,1]。
                local = (u(k) - u_node(idx)) / denom;
            end

            % 分段线性路径 q(u) 在当前段内的一阶导数是常量。
            dq_du = seg_vec(idx, :) / (u_node(idx+1) - u_node(idx));

            % 位置：在当前两个路点间按 local 进行线性插值。
            traj.q(k, :)   = path(idx, :) + local * seg_vec(idx, :);

            % 链式法则：
            %   dq/dt   = dq/du * du/dt
            %   d2q/dt2 = dq/du * d2u/dt2
            % 当前每段 q(u) 为线性函数，所以段内 d2q/du2=0。
            traj.qd(k, :)  = dq_du * ds_dt(k);
            traj.qdd(k, :) = dq_du * d2s_dt2(k);
        end

        % 显式覆盖边界值，消除浮点计算留下的极小误差，
        % 并严格满足课程测试对首末状态的要求。
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
