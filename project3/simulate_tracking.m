function result = simulate_tracking(traj, scene, params, mode)
%SIMULATE_TRACKING 跟踪仿真主循环（运动学 / 简化动力学双模式）
%
%   result = simulate_tracking(traj, scene, params, mode)
%
%   输入：
%       traj   : 由 generate_trajectory 返回的轨迹结构体
%       scene  : 场景结构体（用于绘制障碍）
%       params : 机械臂参数结构体
%       mode   : 'kinematic'（默认）或 'dynamic'
%                  - 'kinematic'：直接把 traj.q 作为实际关节序列
%                  - 'dynamic' ：用 pd_controller 闭环跟踪 traj，
%                                在简化等效惯性 M_eff = I 下用前向 Euler 积分
%
%   输出：
%       result : 结构体，字段如下
%                  result.t         1×K 时间序列
%                  result.q         K×n 实际关节序列
%                  result.q_des     K×n 期望关节序列（即 traj.q）
%                  result.err_pos   K×1 末端位置跟踪误差（基坐标系，欧氏距离）
%                  result.err_q     K×n 关节角误差（q_des − q）
%                  result.mode      'kinematic' 或 'dynamic'
%
%   说明：
%   本函数为老师提供的完整实现。'dynamic' 模式下假定等效惯性矩阵
%   为单位阵、忽略科氏 / 重力 / 摩擦，便于学生在不依赖 RTB
%   动力学参数（质量、惯量张量）的条件下观察 PD 增益的影响。
%   动画通过 RTB 的 SerialLink.plot 实现，每隔 plot_every 帧刷新一次。

    if nargin < 4 || isempty(mode)
        mode = 'kinematic';
    end

    % K 为时间采样点数，n 为机器人关节数。
    K = numel(traj.t);
    n = params.n;

    % 预分配输出数组，避免在循环中动态增长矩阵。
    result.t       = traj.t;
    result.q_des   = traj.q;
    result.q       = zeros(K, n);
    result.err_pos = zeros(K, 1);
    result.err_q   = zeros(K, n);
    result.mode    = mode;

    % ---------------------------------------------------------------------
    % 主循环
    % ---------------------------------------------------------------------
    switch lower(mode)
        case 'kinematic'
            % 运动学模式：q(t) 直接来自轨迹（理想跟踪）
            % 不模拟惯性和控制误差，主要检查几何路径、轨迹和动画。
            result.q = traj.q;

        case 'dynamic'
            % 简化动力学模式：q'' = tau（M_eff = I）
            % 默认 PD 增益（关节空间对角）
            % 对单位惯性二阶模型，Kp=100、Kd=20 近似临界阻尼配置。
            gains.Kp = 100 * ones(1, n);
            gains.Kd =  20 * ones(1, n);

            % 初始实际位置取轨迹起点，初始实际速度设为 0。
            q  = traj.q(1, :);
            qd = zeros(1, n);

            result.q(1, :) = q;

            for k = 1:K-1
                dt = traj.t(k+1) - traj.t(k);

                % 根据实际状态与当前时刻的期望状态计算控制量。
                tau = pd_controller(q, qd, ...
                                    traj.q(k, :), traj.qd(k, :), ...
                                    traj.qdd(k, :), params, gains);

                % 教学简化模型令 M_eff=I，并忽略科氏力、重力和摩擦，
                % 所以 qdd=tau。真实机器人中 tau 通常是力矩，不能直接等同。
                qdd = tau;

                % 离散积分：先更新速度，再用新速度更新位置。
                % dt 越小，数值仿真通常越稳定、越接近连续系统。
                qd = qd + qdd * dt;
                q  = q  + qd  * dt;

                % 控制超调可能令仿真关节越界，这里做保护性裁剪。
                % 更真实的模型还应处理撞限位后的速度和冲击。
                if isfield(params, 'qlim') && ~isempty(params.qlim)
                    q = min(max(q, params.qlim(:, 1)'), params.qlim(:, 2)');
                end

                result.q(k+1, :) = q;
            end

        otherwise
            error('未知的仿真模式 ''%s''，可选 ''kinematic'' 或 ''dynamic''。', mode);
    end

    % ---------------------------------------------------------------------
    % 误差统计
    % ---------------------------------------------------------------------
    % 关节误差统一按“期望值 - 实际值”定义。
    result.err_q = result.q_des - result.q;

    for k = 1:K
        % 末端误差不能简单用关节角误差代替，需要分别做 FK 后比较位置。
        T_des = forward_kinematics(result.q_des(k, :), params);
        T_act = forward_kinematics(result.q(k,     :), params);
        result.err_pos(k) = norm(T_des(1:3, 4) - T_act(1:3, 4));
    end

    % ---------------------------------------------------------------------
    % RTB 动画
    % ---------------------------------------------------------------------
    try
        robot = build_rtb_robot(params);
    catch ME
        warning('build_rtb_robot 调用失败：%s\n跳过动画。', ME.message);
        return;
    end

    % ---------- 先算自适应 workspace ----------
    % 取轨迹所有 frame 原点 + 障碍球外接盒 + base，外扩 padding。
    % workspace 太小会裁掉机械臂，太大又会让主体显示得过小。
    all_pts = zeros(K * (params.n + 1), 3);
    for k = 1:K
        [~, T_all_k] = forward_kinematics(result.q(k, :), params);
        rows = (k - 1) * (params.n + 1) + (1 : params.n + 1);
        all_pts(rows, :) = squeeze(T_all_k(1:3, 4, :))';
    end
    all_pts = [all_pts; 0, 0, 0];   % 将机器人底座也纳入显示范围
    if isfield(scene, 'obstacles') && ~isempty(scene.obstacles)
        % 球心坐标加减半径，得到障碍球包围盒的下角和上角。
        obs_lo = scene.obstacles(:, 1:3) - scene.obstacles(:, 4);
        obs_hi = scene.obstacles(:, 1:3) + scene.obstacles(:, 4);
        all_pts = [all_pts; obs_lo; obs_hi];
    end
    pad = 100;   % mm
    ws_min = min(all_pts, [], 1) - pad;
    ws_max = max(all_pts, [], 1) + pad;
    workspace = [ws_min(1), ws_max(1), ws_min(2), ws_max(2), ws_min(3), ws_max(3)];

    figure('Name', sprintf('Project 3 跟踪仿真 - %s / %s', params.name, mode));
    hold on;

    % 1) 自绘末端轨迹（独立于 RTB trail，一次画完整条蓝线）
    % 预先计算整条线，避免 RTB 不同版本对 trail 参数支持不一致。
    ee_pos = zeros(K, 3);
    for k = 1:K
        T_k = forward_kinematics(result.q(k, :), params);
        ee_pos(k, :) = T_k(1:3, 4)';
    end
    plot3(ee_pos(:, 1), ee_pos(:, 2), ee_pos(:, 3), 'b-', 'LineWidth', 1.5);

    % 2) 障碍球：sphere 返回单位球网格，再缩放并平移到实际位置。
    if isfield(scene, 'obstacles') && ~isempty(scene.obstacles)
        [Xs, Ys, Zs] = sphere(20);
        for i = 1:size(scene.obstacles, 1)
            cx = scene.obstacles(i, 1);
            cy = scene.obstacles(i, 2);
            cz = scene.obstacles(i, 3);
            r  = scene.obstacles(i, 4);
            surf(r*Xs + cx, r*Ys + cy, r*Zs + cz, ...
                 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [1 0.4 0.4]);
        end
    end

    % 3) 锁定 axis 到 workspace（RTB plot 调用前后都重锁一次）
    % daspect([1 1 1]) 保证三个坐标方向比例一致，球不会显示成椭球。
    xlim(workspace(1:2)); ylim(workspace(3:4)); zlim(workspace(5:6));
    set(gca, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
    daspect([1 1 1]);

    % 4) RTB 动画（不带 'trail'，蓝线已在第 1 步画好）
    % 最多抽取约 60 帧展示，避免 K 很大时动画过慢。
    plot_every = max(1, round(K / 60));
    idx_plot = unique([1 : plot_every : K, K]);
    robot.plot(result.q(idx_plot, :), 'fps', 20, ...
               'noshadow', 'workspace', workspace);

    % 5) 动画结束后再 reset，防止 plot 内部把 axis 改掉
    xlim(workspace(1:2)); ylim(workspace(3:4)); zlim(workspace(5:6));
    set(gca, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
    hold off;
end
