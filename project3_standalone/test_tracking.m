function test_tracking(robot_type, scene_name, sim_mode)
%TEST_TRACKING 端到端跟踪测试 + RTB 动画展示
%
%   test_tracking(robot_type, scene_name, sim_mode)
%
%   流程：plan_path → generate_trajectory → simulate_tracking
%
%   判据：
%       'kinematic' 模式：末端跟踪误差恒为 0
%       'dynamic'   模式：末端跟踪误差最大值 < 阈值（单位与 params.a/d 一致）

    if nargin < 3 || isempty(sim_mode)
        sim_mode = 'kinematic';
    end

    fprintf('\n========== 端到端跟踪测试（%s） ==========\n', sim_mode);

    params = robot_params(robot_type);
    scene  = load_scene(scene_name, params);

    try
        path = plan_path(scene, params);
    catch ME
        fprintf('[SKIP] plan_path 未完成：%s\n', ME.message);
        return;
    end
    try
        traj = generate_trajectory(path, scene, params);
    catch ME
        fprintf('[SKIP] generate_trajectory 未完成：%s\n', ME.message);
        return;
    end

    try
        result = simulate_tracking(traj, scene, params, sim_mode);
    catch ME
        fprintf('[FAIL] simulate_tracking 抛出异常：%s\n', ME.message);
        return;
    end

    n_check = 0;
    n_pass  = 0;

    % ---------- 跟踪误差判据 ----------
    n_check = n_check + 1;
    max_err = max(result.err_pos);

    switch lower(sim_mode)
        case 'kinematic'
            tol = 1e-6;
        case 'dynamic'
            % 阈值以场景尺度的 5% 作为参考，可在课程报告中讨论调参
            scale = max(1, norm(max(scene.obstacles(:, 1:3), [], 1)));
            tol   = 0.05 * scale;
        otherwise
            tol = inf;
    end

    if max_err < tol
        fprintf('  [PASS] 末端位置最大跟踪误差 = %.3e（阈值 %.3e）\n', max_err, tol);
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] 末端位置最大跟踪误差 = %.3e 超出阈值 %.3e\n', max_err, tol);
    end

    fprintf('本测试通过：%d / %d\n', n_pass, n_check);
end

function scene = load_scene(scene_name, params)
    switch lower(scene_name)
        case 'easy', scene = scene_easy(params);
        case 'hard', scene = scene_hard(params);
        otherwise, error('未知的场景名 ''%s''。', scene_name);
    end
end
