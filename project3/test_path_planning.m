function test_path_planning(robot_type, scene_name)
%TEST_PATH_PLANNING 校验 plan_path 输出的合法性
%
%   test_path_planning(robot_type, scene_name)
%
%   判据：
%       1. path 维度合法（M×n，M ≥ 2）
%       2. path(1,:)   ≈ scene.q_start
%       3. path(end,:) ≈ scene.q_goal
%       4. 每个路点在 params.qlim 内
%       5. 相邻路点关节空间线性插值采样下，末端不与任何球障碍发生碰撞

    fprintf('\n========== 路径规划合法性测试 ==========\n');

    params = robot_params(robot_type);
    scene  = load_scene(scene_name, params);

    try
        path = plan_path(scene, params);
    catch ME
        fprintf('[FAIL] plan_path 抛出异常：%s\n', ME.message);
        return;
    end

    n_check     = 0;
    n_pass      = 0;
    tol_angle   = 1e-6;
    interp_step = 20;   % 相邻路点之间插值采样数

    % ---------- 1. 维度 ----------
    n_check = n_check + 1;
    if ismatrix(path) && size(path, 2) == params.n && size(path, 1) >= 2 ...
            && all(isfinite(path(:)))
        fprintf('  [PASS] path 维度合法：%d × %d\n', size(path, 1), size(path, 2));
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] path 维度非法或含 NaN：size = [%s]\n', num2str(size(path)));
        fprintf('         本测试通过：%d / %d\n', n_pass, n_check);
        return;
    end

    % ---------- 2. 起点匹配 ----------
    n_check = n_check + 1;
    if norm(path(1, :) - scene.q_start) < tol_angle
        fprintf('  [PASS] path(1,:) 与 scene.q_start 匹配\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] path(1,:) 与 scene.q_start 不匹配（误差 = %.3e）\n', ...
                norm(path(1, :) - scene.q_start));
    end

    % ---------- 3. 终点匹配 ----------
    n_check = n_check + 1;
    if norm(path(end, :) - scene.q_goal) < tol_angle
        fprintf('  [PASS] path(end,:) 与 scene.q_goal 匹配\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] path(end,:) 与 scene.q_goal 不匹配（误差 = %.3e）\n', ...
                norm(path(end, :) - scene.q_goal));
    end

    % ---------- 4. 关节限位 ----------
    n_check = n_check + 1;
    in_qlim = true;
    for i = 1:size(path, 1)
        if any(path(i, :) < params.qlim(:, 1)' - 1e-9) ...
                || any(path(i, :) > params.qlim(:, 2)' + 1e-9)
            in_qlim = false;
            fprintf('  [FAIL] 路点 %d 越出 qlim：q = [%s]\n', i, num2str(path(i, :)));
            break;
        end
    end
    if in_qlim
        fprintf('  [PASS] 所有路点在 qlim 内\n');
        n_pass = n_pass + 1;
    end

    % ---------- 5. 避障（全臂：所有连杆线段 vs 球障）----------
    n_check = n_check + 1;
    collision = false;
    for i = 1:size(path, 1)-1
        for s = linspace(0, 1, interp_step)
            q = (1-s) * path(i, :) + s * path(i+1, :);
            [~, T_all] = forward_kinematics(q, params);
            pts = squeeze(T_all(1:3, 4, :))';   % (n+1) × 3
            for k = 1:size(scene.obstacles, 1)
                cx = scene.obstacles(k, 1:3);
                r  = scene.obstacles(k, 4);
                for j = 1:size(pts, 1) - 1
                    d = seg_point_dist(pts(j, :), pts(j+1, :), cx);
                    if d < r
                        collision = true;
                        fprintf(['  [FAIL] 段 %d→%d 在 s=%.2f 处连杆 %d 进入障碍 %d ' ...
                                 '(球心 [%s], r=%.1f, 距连杆 %.1f mm)\n'], ...
                                i, i+1, s, j, k, num2str(cx, '%.1f '), r, d);
                        break;
                    end
                end
                if collision, break; end
            end
            if collision, break; end
        end
        if collision, break; end
    end
    if ~collision
        fprintf('  [PASS] 路径采样下整条机械臂未与任何障碍发生碰撞\n');
        n_pass = n_pass + 1;
    end

    fprintf('本测试通过：%d / %d\n', n_pass, n_check);
end

function scene = load_scene(scene_name, params)
    switch lower(scene_name)
        case 'easy'
            scene = scene_easy(params);
        case 'hard'
            scene = scene_hard(params);
        otherwise
            error('未知的场景名 ''%s''，可选 ''easy'' 或 ''hard''。', scene_name);
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
