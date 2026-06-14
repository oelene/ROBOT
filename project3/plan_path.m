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
    % 统一整理为 1×n 行向量。路径矩阵规定每一行是一组关节构型。
    q_start = scene.q_start(:)';
    q_goal  = scene.q_goal(:)';
    n = params.n;

    if numel(q_start) ~= n || numel(q_goal) ~= n
        error('路径规划失败：scene.q_start / scene.q_goal 维度与 params.n 不一致。');
    end

    % 获取每个关节的 [下限, 上限]。如果参数中没有提供，就使用
    % [-pi,pi] 作为通用默认值。
    qlim = get_qlim(params, n);

    % clamp 是数值保护，不应被理解为“越限也没关系”。正常场景的
    % 起终点本来就应位于 qlim 内；这里主要处理极小浮点误差。
    q_start = clamp_to_qlim(q_start, qlim);
    q_goal  = clamp_to_qlim(q_goal,  qlim);

    % 起点或终点自身碰撞时，不可能存在满足题目要求的连接路径，
    % 因此在进入随机搜索前立即报错。
    if ~is_state_valid(q_start, scene, params, qlim)
        error('路径规划失败：起点构型不合法或与障碍碰撞。');
    end
    if ~is_state_valid(q_goal, scene, params, qlim)
        error('路径规划失败：终点构型不合法或与障碍碰撞。');
    end

    % 两个端点安全不代表中间过程安全。每条关节空间边均匀采样
    % 25 次，并对每个中间构型执行全臂碰撞检测。
    edge_samples = 25;

    % 分层策略：先尝试最简单、最短且确定性的关节空间直线。
    % 只有直线碰撞时，才启动计算量更大的双向 RRT。
    if is_edge_valid(q_start, q_goal, scene, params, qlim, edge_samples)
        path = [q_start; q_goal];
    else
        path = plan_with_birrt(q_start, q_goal, scene, params, qlim, edge_samples);
    end

    % 双向树在循环中会交换角色，合并后的路径方向可能为 goal->start。
    % 比较两端谁更接近起点，如有需要就翻转整条路径。
    if norm(path(1, :) - q_start) > norm(path(end, :) - q_start)
        path = flipud(path);
    end
    path(1, :) = q_start;
    path(end, :) = q_goal;

    if any(~isfinite(path(:)))
        error('路径规划失败：plan_path.m 中的 TODO 尚未完成。');
    end
end

function path = plan_with_birrt(q_start, q_goal, scene, params, qlim, edge_samples)
%PLAN_WITH_BIRRT 使用双向 RRT-Connect 搜索无碰撞关节空间路径。
%
% 两棵树分别从路径两端生长。每轮让 tree_a 朝采样点扩展一步，
% 再让 tree_b 尽可能连续地朝新节点靠近，随后交换两棵树的角色。

    % 固定随机种子，使相同场景下的搜索结果可复现。
    rng(3, 'twister');  % 固定随机种子，便于复现实验结果

    n = params.n;
    max_iter = 6000;
    % 步长单位是关节空间欧氏距离（rad）。将其限制在 [0.12,0.35]：
    % 太小会使树增长很慢，太大则容易跨入障碍或错过狭窄通道。
    step = min(0.35, max(0.12, norm(q_goal - q_start) / 5));
    connect_tol = step;

    % q 的每一行是一个树节点；parent(k) 是第 k 个节点的父节点编号。
    % 根节点没有父节点，用 0 表示。
    tree_a.q = q_start;
    tree_a.parent = 0;
    tree_b.q = q_goal;
    tree_b.parent = 0;

    for iter = 1:max_iter
        % 每 5 次直接采样目标，称为目标偏置；其余时间探索关节空间。
        if mod(iter, 5) == 0
            q_rand = q_goal;
        else
            q_rand = sample_configuration(qlim, q_start, q_goal, iter);
        end

        % 树 A 只向随机目标扩展一个 step。
        [tree_a, idx_new, added] = extend_tree(tree_a, q_rand, step, scene, params, qlim, edge_samples);
        if added
            q_new = tree_a.q(idx_new, :);
            % 树 B 按 RRT-Connect 方式反复扩展，尽量一路连接到 q_new。
            [tree_b, idx_other, reached] = connect_tree(tree_b, q_new, step, connect_tol, ...
                                                        scene, params, qlim, edge_samples);
            if reached
                % 两树相遇后沿 parent 回溯，并用 shortcut 删除多余折点。
                path = merge_trees(tree_a, idx_new, tree_b, idx_other);
                path = shortcut_path(path, scene, params, qlim, edge_samples);
                return;
            end
        end

        % 交换角色，使起点侧和终点侧轮流主动扩展。
        tmp = tree_a;
        tree_a = tree_b;
        tree_b = tmp;
    end

    % 随机搜索达到上限后，尝试若干确定性中间构型作为兜底。
    % 这是课程场景的工程补强，不代表任意障碍布局下都一定成功。
    path = try_manual_detours(q_start, q_goal, scene, params, qlim, edge_samples);
    if isempty(path)
        error('路径规划失败：RRT 在迭代上限内未找到无碰撞路径。');
    end
end

function qlim = get_qlim(params, n)
%GET_QLIM 返回 n 个关节的上下限矩阵。
    if isfield(params, 'qlim') && isequal(size(params.qlim), [n, 2])
        qlim = params.qlim;
    else
        qlim = repmat([-pi, pi], n, 1);
    end
end

function q = clamp_to_qlim(q, qlim)
%CLAMP_TO_QLIM 把每个关节值裁剪到自己的合法区间。
    q = min(max(q, qlim(:, 1)'), qlim(:, 2)');
end

function q = sample_configuration(qlim, q_start, q_goal, iter)
%SAMPLE_CONFIGURATION 在关节限位内产生候选采样构型。
%
% 大部分迭代使用全局均匀随机采样；部分迭代围绕起终点中间构型
% 对关键关节施加偏移，增加发现抬臂、收臂等绕障通道的机会。
    n = size(qlim, 1);
    span = qlim(:, 2)' - qlim(:, 1)';

    if mod(iter, 7) == 0
        % 结构化样本围绕路径中间构型产生。
        base = (q_start + q_goal) / 2;
        offsets = zeros(1, n);
        if n >= 2, offsets(2) = ((-1)^iter) * pi/4; end
        if n >= 3, offsets(3) = ((-1)^(floor(iter/7))) * pi/3; end
        if n >= 5, offsets(5) = ((-1)^(floor(iter/11))) * pi/5; end
        q = base + offsets;
    else
        % 将 [0,1] 均匀随机数线性缩放到每个关节的限位区间。
        q = qlim(:, 1)' + rand(1, n) .* span;
    end

    q = clamp_to_qlim(q, qlim);
end

function [tree, idx_new, added] = extend_tree(tree, q_target, step, scene, params, qlim, edge_samples)
%EXTEND_TREE 从树中最近节点朝 q_target 扩展一步。
%
% added=false 表示新状态不合法，或最近节点到新状态的边发生碰撞。

    % 在已有节点中寻找关节空间距离 q_target 最近者。
    idx_near = nearest_node(tree.q, q_target);
    q_near = tree.q(idx_near, :);
    q_new = steer(q_near, q_target, step);
    q_new = clamp_to_qlim(q_new, qlim);

    % “点合法”和“边合法”缺一不可：前者检查 q_new，后者还检查
    % q_near 到 q_new 的整个插值运动。
    added = is_state_valid(q_new, scene, params, qlim) && ...
            is_edge_valid(q_near, q_new, scene, params, qlim, edge_samples);
    if added
        % 保存父节点编号，搜索成功后才能沿 parent 回溯路径。
        tree.q(end+1, :) = q_new;
        tree.parent(end+1, 1) = idx_near;
        idx_new = size(tree.q, 1);
    else
        idx_new = idx_near;
    end
end

function [tree, idx_new, reached] = connect_tree(tree, q_target, step, connect_tol, scene, params, qlim, edge_samples)
%CONNECT_TREE 连续朝 q_target 扩展，直到被挡住或成功连接。
%
% 与 extend_tree 的单步扩展不同，本函数在 while 中可加入多个节点。
    reached = false;
    idx_new = nearest_node(tree.q, q_target);

    while true
        [tree, idx_new, added] = extend_tree(tree, q_target, step, scene, params, qlim, edge_samples);
        if ~added
            return;
        end
        % 足够接近后还需检查最后一小段是否可安全直连。
        if norm(tree.q(idx_new, :) - q_target) <= connect_tol && ...
                is_edge_valid(tree.q(idx_new, :), q_target, scene, params, qlim, edge_samples)
            % 加入精确目标点，避免两棵树之间留下数值缝隙。
            tree.q(end+1, :) = q_target;
            tree.parent(end+1, 1) = idx_new;
            idx_new = size(tree.q, 1);
            reached = true;
            return;
        end
    end
end

function q_new = steer(q_from, q_to, step)
%STEER 从 q_from 朝 q_to 前进一步，单次距离不超过 step。
    delta = q_to - q_from;
    dist = norm(delta);
    if dist <= step
        q_new = q_to;
    else
        % delta/dist 是六维关节空间中的单位方向向量。
        q_new = q_from + step * delta / dist;
    end
end

function idx = nearest_node(nodes, q)
%NEAREST_NODE 返回 nodes 中与 q 欧氏距离最小的行号。
    % 比较距离平方即可确定最近点，无需执行 sqrt。
    d = sum((nodes - q).^2, 2);
    [~, idx] = min(d);
end

function path = merge_trees(tree_a, idx_a, tree_b, idx_b)
%MERGE_TREES 回溯两棵已经连接的树并拼成完整路径。
    path_a = trace_path(tree_a, idx_a);
    path_b = trace_path(tree_b, idx_b);
    % path_a 为“根 A -> 连接点”，path_b 为“根 B -> 连接点”。
    % 第二段翻转后才是“连接点 -> 根 B”。
    path = [path_a; flipud(path_b)];

    if norm(path(1, :) - tree_b.q(1, :)) < norm(path(1, :) - tree_a.q(1, :))
        path = flipud(path);
    end
end

function path = trace_path(tree, idx)
%TRACE_PATH 从节点 idx 沿 parent 指针回溯到树根。
    path = tree.q(idx, :);
    while tree.parent(idx) ~= 0
        idx = tree.parent(idx);
        % 父节点插到最前面，最终得到“根 -> idx”的正确顺序。
        path = [tree.q(idx, :); path]; %#ok<AGROW>
    end
end

function path = shortcut_path(path, scene, params, qlim, edge_samples)
%SHORTCUT_PATH 删除可被一条无碰撞长边跨过的中间路点。
    i = 1;
    while i < size(path, 1) - 1
        shortened = false;
        % 从最远的 j 开始尝试，优先一次删除更多中间点。
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
%TRY_MANUAL_DETOURS 随机搜索失败后的启发式兜底。
%
% 通过改变肩、肘、腕关节尝试从障碍上方、下方或侧面绕行。
% 每一条候选边仍需经过完整碰撞检测，不会被无条件接受。
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
%IS_STATE_VALID 检查关节角是否有限、在限位内且全臂无碰撞。
    ok = all(isfinite(q)) && all(q >= qlim(:, 1)' - 1e-9) && all(q <= qlim(:, 2)' + 1e-9);
    if ok
        ok = ~is_collision(q, scene, params);
    end
end

function ok = is_edge_valid(q1, q2, scene, params, qlim, edge_samples)
%IS_EDGE_VALID 离散检查 q1 到 q2 的关节空间直线是否全程安全。
%
% q(s)=(1-s)q1+s*q2。离散采样不是严格的连续碰撞证明，但可在
% 课程场景中取得检测安全性与计算量之间的折中。
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
%IS_COLLISION 检查构型 q 下任意连杆是否进入任意球形障碍。
    collision = false;
    if ~isfield(scene, 'obstacles') || isempty(scene.obstacles)
        return;
    end

    % 正运动学给出所有坐标系原点，相邻原点间的线段近似一根连杆。
    [~, T_all] = forward_kinematics(q, params);
    pts = squeeze(T_all(1:3, 4, :))';

    % 遍历“每个障碍球 × 每根连杆”。
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
%SEG_POINT_DIST 计算点 C 到有限线段 AB 的最短欧氏距离。
    AB = B - A;
    L2 = AB * AB';
    if L2 < 1e-12
        % A、B 几乎重合时线段退化为一点，避免除以接近 0 的 L2。
        d = norm(A - C);
        return;
    end
    % 先求 C 在无限直线 AB 上的投影参数，再裁剪到 [0,1]：
    % t=0 是 A，t=1 是 B，0<t<1 位于线段内部。
    t = max(0, min(1, dot(C - A, AB) / L2));
    P = A + t * AB;
    d = norm(P - C);
end
