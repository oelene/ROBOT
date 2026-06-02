function test_extra_path_cases(robot_type)
%TEST_EXTRA_PATH_CASES 额外路径规划/轨迹生成测试用例
%
%   test_extra_path_cases()
%   test_extra_path_cases('SR3')
%
%   本文件用于在 README 给定 easy/hard 场景之外，额外检查 plan_path
%   和 generate_trajectory 在多组 SR3 自定义场景中的表现。

    if nargin < 1 || isempty(robot_type)
        robot_type = 'SR3';
    end

    fprintf('\n========== 额外路径规划测试（%s） ==========\n', robot_type);

    params = robot_params(robot_type);
    scenes = extra_path_scenes(params);

    n_case = numel(scenes);
    n_pass = 0;

    for i = 1:n_case
        scene = scenes(i);
        fprintf('\n[%d/%d] %s\n', i, n_case, scene.name);

        straight_ok = is_edge_valid(scene.q_start, scene.q_goal, scene, params, 25);
        if straight_ok
            fprintf('  直线路径检测：可行\n');
        else
            fprintf('  直线路径检测：不可行，需要规划器绕障\n');
        end

        try
            path = plan_path(scene, params);
            traj = generate_trajectory(path, scene, params);
        catch ME
            fprintf('  [FAIL] 函数执行失败：%s\n', ME.message);
            continue;
        end

        [ok_path, msg_path] = check_path(path, scene, params);
        [ok_traj, msg_traj] = check_traj(traj, path, scene, params);

        if ok_path && ok_traj
            fprintf('  [PASS] path = %d × %d，traj.K = %d\n', ...
                    size(path, 1), size(path, 2), numel(traj.t));
            n_pass = n_pass + 1;
        else
            fprintf('  [FAIL] %s %s\n', msg_path, msg_traj);
        end
    end

    fprintf('\n额外测试通过：%d / %d\n', n_pass, n_case);
end

function [ok, msg] = check_path(path, scene, params)
    ok = false;
    msg = '';

    if ~ismatrix(path) || size(path, 2) ~= params.n || size(path, 1) < 2 || any(~isfinite(path(:)))
        msg = 'path 维度非法或含 NaN。';
        return;
    end
    if norm(path(1, :) - scene.q_start) > 1e-6 || norm(path(end, :) - scene.q_goal) > 1e-6
        msg = 'path 首末点不匹配。';
        return;
    end
    for i = 1:size(path, 1)
        if any(path(i, :) < params.qlim(:, 1)' - 1e-9) || any(path(i, :) > params.qlim(:, 2)' + 1e-9)
            msg = sprintf('path 第 %d 个路点存在关节限位异常。', i);
            return;
        end
    end

    for i = 1:size(path, 1)-1
        if ~is_edge_valid(path(i, :), path(i+1, :), scene, params, 25)
            msg = sprintf('path 第 %d 段发生碰撞。', i);
            return;
        end
    end

    ok = true;
end

function [ok, msg] = check_traj(traj, path, scene, params)
    ok = false;
    msg = '';
    K = round(scene.T_total / scene.dt) + 1;

    if ~isfield(traj, 't') || ~isfield(traj, 'q') || ~isfield(traj, 'qd') || ~isfield(traj, 'qdd')
        msg = 'traj 字段不完整。';
        return;
    end
    if ~isequal(size(traj.t), [1, K]) || ~isequal(size(traj.q), [K, params.n]) ...
            || ~isequal(size(traj.qd), [K, params.n]) || ~isequal(size(traj.qdd), [K, params.n])
        msg = 'traj 维度不正确。';
        return;
    end
    if norm(traj.q(1, :) - path(1, :)) > 1e-6 || norm(traj.q(end, :) - path(end, :)) > 1e-6
        msg = 'traj 首末位置不匹配。';
        return;
    end
    if norm(traj.qd(1, :)) > 1e-6 || norm(traj.qd(end, :)) > 1e-6 ...
            || norm(traj.qdd(1, :)) > 1e-6 || norm(traj.qdd(end, :)) > 1e-6
        msg = 'traj 首末速度或加速度不为 0。';
        return;
    end

    ok = true;
end

function ok = is_edge_valid(q1, q2, scene, params, n_sample)
    ok = true;
    for s = linspace(0, 1, n_sample)
        q = (1 - s) * q1 + s * q2;
        if is_collision(q, scene, params)
            ok = false;
            return;
        end
    end
end

function collision = is_collision(q, scene, params)
    collision = false;
    if isempty(scene.obstacles)
        return;
    end

    [~, T_all] = forward_kinematics(q, params);
    pts = squeeze(T_all(1:3, 4, :))';

    for k = 1:size(scene.obstacles, 1)
        center = scene.obstacles(k, 1:3);
        radius = scene.obstacles(k, 4);
        for j = 1:size(pts, 1)-1
            if seg_point_dist(pts(j, :), pts(j+1, :), center) < radius
                collision = true;
                return;
            end
        end
    end
end

function d = seg_point_dist(A, B, C)
    AB = B - A;
    L2 = AB * AB';
    if L2 < 1e-12
        d = norm(A - C);
        return;
    end
    t = max(0, min(1, dot(C - A, AB) / L2));
    P = A + t * AB;
    d = norm(P - C);
end
