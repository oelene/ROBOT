function path = plan_path(scene, params)
%PLAN_PATH 在给定场景下生成一条由起点到终点的关节空间路径
%
%   path = plan_path(scene, params)
%
%   输入：
%       scene  : 场景结构体（见 scene_easy.m / scene_hard.m）
%                必须使用的字段：
%                  scene.q_start  1×n 起始关节角
%                  scene.q_goal   1×n 目标关节角
%                  scene.obstacles M×4 球形障碍 [cx, cy, cz, r]
%       params : 由 robot_params() 返回的机械臂参数结构体
%
%   输出：
%       path   : M×n 关节空间路点矩阵
%                - 必须满足 path(1,  :) == scene.q_start
%                          path(end,:) == scene.q_goal
%                - 中间行数 M ≥ 2 由所选算法决定
%                - 每个路点须在 params.qlim 内
%                - 相邻路点连线在任务空间下不应穿过 scene.obstacles
%
%   说明：
%   本文件为开放性任务。算法可由小组自行选定（直线插值、Bezier、
%   APF、PRM、RRT 等），实现后请在课程设计报告中说明：
%       1. 所选算法名称与适用场景；
%       2. 关键参数（步长、迭代上限、采样数 等）；
%       3. 在 scene_easy / scene_hard 下的运行结果与失败情形。

    % =====================================================================
    % TODO: 实现你们组的路径规划算法
    % ---------------------------------------------------------------------
    %   要求：
    %     - 输出 path 必须为 M×n 数值矩阵（n = params.n）
    %     - 首行为 scene.q_start，末行为 scene.q_goal
    %     - 中间点须满足 qlim 约束与避障约束
    %
    %   未填时返回 NaN 矩阵，下游测试会精确报错指向本 TODO。
    % =====================================================================
    q_start = scene.q_start(:)';
    q_goal  = scene.q_goal(:)';
    n = params.n;

    if numel(q_start) ~= n || numel(q_goal) ~= n
        error('路径规划失败：scene.q_start / scene.q_goal 维度与 params.n 不一致。');
    end

    qlim = get_qlim(params, n);
    q_start = clamp_to_qlim(q_start, qlim);
    q_goal  = clamp_to_qlim(q_goal,  qlim);

    if ~is_state_valid(q_start, scene, params, qlim)
        error('路径规划失败：起点构型不合法或与障碍碰撞。');
    end
    if ~is_state_valid(q_goal, scene, params, qlim)
        error('路径规划失败：终点构型不合法或与障碍碰撞。');
    end

    edge_samples = 25;
    if is_edge_valid(q_start, q_goal, scene, params, qlim, edge_samples)
        path = [q_start; q_goal];
    else
        path = plan_with_birrt(q_start, q_goal, scene, params, qlim, edge_samples);
    end

    if any(~isfinite(path(:)))
        error('路径规划失败：plan_path.m 中的 TODO 尚未完成。');
    end
end

function path = plan_with_birrt(q_start, q_goal, scene, params, qlim, edge_samples)
    rng(3, 'twister');  % 固定随机种子，便于复现实验结果

    n = params.n;
    max_iter = 6000;
    step = min(0.35, max(0.12, norm(q_goal - q_start) / 5));
    connect_tol = step;

    tree_a.q = q_start;
    tree_a.parent = 0;
    tree_b.q = q_goal;
    tree_b.parent = 0;

    for iter = 1:max_iter
        if mod(iter, 5) == 0
            q_rand = q_goal;
        else
            q_rand = sample_configuration(qlim, q_start, q_goal, iter);
        end

        [tree_a, idx_new, added] = extend_tree(tree_a, q_rand, step, scene, params, qlim, edge_samples);
        if added
            q_new = tree_a.q(idx_new, :);
            [tree_b, idx_other, reached] = connect_tree(tree_b, q_new, step, connect_tol, ...
                                                        scene, params, qlim, edge_samples);
            if reached
                path = merge_trees(tree_a, idx_new, tree_b, idx_other);
                path = shortcut_path(path, scene, params, qlim, edge_samples);
                return;
            end
        end

        tmp = tree_a;
        tree_a = tree_b;
        tree_b = tmp;
    end

    % 确定性候选路点作为兜底，适合本项目两个给定场景。
    path = try_manual_detours(q_start, q_goal, scene, params, qlim, edge_samples);
    if isempty(path)
        error('路径规划失败：RRT 在迭代上限内未找到无碰撞路径。');
    end
end

function qlim = get_qlim(params, n)
    if isfield(params, 'qlim') && isequal(size(params.qlim), [n, 2])
        qlim = params.qlim;
    else
        qlim = repmat([-pi, pi], n, 1);
    end
end

function q = clamp_to_qlim(q, qlim)
    q = min(max(q, qlim(:, 1)'), qlim(:, 2)');
end

function q = sample_configuration(qlim, q_start, q_goal, iter)
    n = size(qlim, 1);
    span = qlim(:, 2)' - qlim(:, 1)';

    if mod(iter, 7) == 0
        base = (q_start + q_goal) / 2;
        offsets = zeros(1, n);
        if n >= 2, offsets(2) = ((-1)^iter) * pi/4; end
        if n >= 3, offsets(3) = ((-1)^(floor(iter/7))) * pi/3; end
        if n >= 5, offsets(5) = ((-1)^(floor(iter/11))) * pi/5; end
        q = base + offsets;
    else
        q = qlim(:, 1)' + rand(1, n) .* span;
    end

    q = clamp_to_qlim(q, qlim);
end

function [tree, idx_new, added] = extend_tree(tree, q_target, step, scene, params, qlim, edge_samples)
    idx_near = nearest_node(tree.q, q_target);
    q_near = tree.q(idx_near, :);
    q_new = steer(q_near, q_target, step);
    q_new = clamp_to_qlim(q_new, qlim);

    added = is_state_valid(q_new, scene, params, qlim) && ...
            is_edge_valid(q_near, q_new, scene, params, qlim, edge_samples);
    if added
        tree.q(end+1, :) = q_new;
        tree.parent(end+1, 1) = idx_near;
        idx_new = size(tree.q, 1);
    else
        idx_new = idx_near;
    end
end

function [tree, idx_new, reached] = connect_tree(tree, q_target, step, connect_tol, scene, params, qlim, edge_samples)
    reached = false;
    idx_new = nearest_node(tree.q, q_target);

    while true
        [tree, idx_new, added] = extend_tree(tree, q_target, step, scene, params, qlim, edge_samples);
        if ~added
            return;
        end
        if norm(tree.q(idx_new, :) - q_target) <= connect_tol && ...
                is_edge_valid(tree.q(idx_new, :), q_target, scene, params, qlim, edge_samples)
            tree.q(end+1, :) = q_target;
            tree.parent(end+1, 1) = idx_new;
            idx_new = size(tree.q, 1);
            reached = true;
            return;
        end
    end
end

function q_new = steer(q_from, q_to, step)
    delta = q_to - q_from;
    dist = norm(delta);
    if dist <= step
        q_new = q_to;
    else
        q_new = q_from + step * delta / dist;
    end
end

function idx = nearest_node(nodes, q)
    d = sum((nodes - q).^2, 2);
    [~, idx] = min(d);
end

function path = merge_trees(tree_a, idx_a, tree_b, idx_b)
    path_a = trace_path(tree_a, idx_a);
    path_b = trace_path(tree_b, idx_b);
    path = [path_a; flipud(path_b)];

    if norm(path(1, :) - tree_b.q(1, :)) < norm(path(1, :) - tree_a.q(1, :))
        path = flipud(path);
    end
end

function path = trace_path(tree, idx)
    path = tree.q(idx, :);
    while tree.parent(idx) ~= 0
        idx = tree.parent(idx);
        path = [tree.q(idx, :); path]; %#ok<AGROW>
    end
end

function path = shortcut_path(path, scene, params, qlim, edge_samples)
    i = 1;
    while i < size(path, 1) - 1
        shortened = false;
        for j = size(path, 1):-1:(i+2)
            if is_edge_valid(path(i, :), path(j, :), scene, params, qlim, edge_samples)
                path = [path(1:i, :); path(j:end, :)];
                shortened = true;
                break;
            end
        end
        if ~shortened
            i = i + 1;
        end
    end
end

function path = try_manual_detours(q_start, q_goal, scene, params, qlim, edge_samples)
    n = params.n;
    base = (q_start + q_goal) / 2;
    candidates = {};
    magnitudes = [pi/6, pi/4, pi/3, pi/2];
    joints = unique([2, 3, 5, min(n, 1)]);

    for mag = magnitudes
        for s2 = [-1, 1]
            q_mid = base;
            for j = joints
                q_mid(j) = q_mid(j) + s2 * mag;
            end
            candidates{end+1} = [q_start; clamp_to_qlim(q_mid, qlim); q_goal]; %#ok<AGROW>
        end
        if n >= 3
            q1 = base; q2 = base;
            q1(2) = q1(2) + mag; q1(3) = q1(3) - mag;
            q2(2) = q2(2) - mag; q2(3) = q2(3) + mag;
            candidates{end+1} = [q_start; clamp_to_qlim(q1, qlim); clamp_to_qlim(q2, qlim); q_goal]; %#ok<AGROW>
            candidates{end+1} = [q_start; clamp_to_qlim(q2, qlim); clamp_to_qlim(q1, qlim); q_goal]; %#ok<AGROW>
        end
    end

    path = [];
    for i = 1:numel(candidates)
        p = candidates{i};
        ok = true;
        for k = 1:size(p, 1)-1
            if ~is_edge_valid(p(k, :), p(k+1, :), scene, params, qlim, edge_samples)
                ok = false;
                break;
            end
        end
        if ok
            path = p;
            return;
        end
    end
end

function ok = is_state_valid(q, scene, params, qlim)
    ok = all(isfinite(q)) && all(q >= qlim(:, 1)' - 1e-9) && all(q <= qlim(:, 2)' + 1e-9);
    if ok
        ok = ~is_collision(q, scene, params);
    end
end

function ok = is_edge_valid(q1, q2, scene, params, qlim, edge_samples)
    ok = true;
    for s = linspace(0, 1, edge_samples)
        q = (1 - s) * q1 + s * q2;
        if ~is_state_valid(q, scene, params, qlim)
            ok = false;
            return;
        end
    end
end

function collision = is_collision(q, scene, params)
    collision = false;
    if ~isfield(scene, 'obstacles') || isempty(scene.obstacles)
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
