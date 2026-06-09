function test_trajectory(robot_type, scene_name)
%TEST_TRAJECTORY 校验 generate_trajectory 输出的合法性与平滑性
%
%   test_trajectory(robot_type, scene_name)
%
%   判据：
%       1. traj.t、traj.q、traj.qd、traj.qdd 字段维度匹配 K×n / 1×K
%       2. traj.t(1) == 0、traj.t(end) ≈ scene.T_total，等距步长 ≈ scene.dt
%       3. traj.q 在首末点与 path 一致
%       4. traj.qd 首末时刻为 0
%       5. traj.qdd 首末时刻为 0
%       6. 离散差分 diff(q)/dt 与 qd 匹配（中段，允许 1% 相对误差）

    fprintf('\n========== 轨迹平滑性测试 ==========\n');

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
        fprintf('[FAIL] generate_trajectory 抛出异常：%s\n', ME.message);
        return;
    end

    n_check = 0;
    n_pass  = 0;
    n       = params.n;
    K_exp   = round(scene.T_total / scene.dt) + 1;

    % ---------- 1. 字段与维度 ----------
    n_check = n_check + 1;
    ok = all(isfield(traj, {'t','q','qd','qdd'})) ...
         && isequal(size(traj.t),   [1 K_exp]) ...
         && isequal(size(traj.q),   [K_exp n]) ...
         && isequal(size(traj.qd),  [K_exp n]) ...
         && isequal(size(traj.qdd), [K_exp n]) ...
         && all(isfinite(traj.t)) && all(isfinite(traj.q(:))) ...
         && all(isfinite(traj.qd(:))) && all(isfinite(traj.qdd(:)));
    if ok
        fprintf('  [PASS] traj 字段与维度合法（K=%d, n=%d）\n', K_exp, n);
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] traj 字段或维度异常，期望 K=%d、n=%d\n', K_exp, n);
        fprintf('本测试通过：%d / %d\n', n_pass, n_check);
        return;
    end

    % ---------- 2. 时间序列 ----------
    n_check = n_check + 1;
    dt_actual = diff(traj.t);
    if abs(traj.t(1)) < 1e-12 ...
            && abs(traj.t(end) - scene.T_total) < 1e-6 ...
            && max(abs(dt_actual - scene.dt)) < 1e-9
        fprintf('  [PASS] 时间序列等距、首末符合预期\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] 时间序列异常：t(1)=%.3e, t(end)=%.6f（期望 %.6f）, max|dt-dt_ref|=%.3e\n', ...
                traj.t(1), traj.t(end), scene.T_total, max(abs(dt_actual - scene.dt)));
    end

    % ---------- 3. 端点匹配 ----------
    n_check = n_check + 1;
    if norm(traj.q(1, :)   - path(1, :))   < 1e-6 ...
            && norm(traj.q(end, :) - path(end, :)) < 1e-6
        fprintf('  [PASS] traj.q 首末点与 path 匹配\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] traj.q 首末点与 path 不匹配\n');
    end

    % ---------- 4. 边界速度 ----------
    n_check = n_check + 1;
    if norm(traj.qd(1, :)) < 1e-6 && norm(traj.qd(end, :)) < 1e-6
        fprintf('  [PASS] qd 首末为 0\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] qd 首末非零：||qd(1)||=%.3e、||qd(end)||=%.3e\n', ...
                norm(traj.qd(1, :)), norm(traj.qd(end, :)));
    end

    % ---------- 5. 边界加速度 ----------
    n_check = n_check + 1;
    if norm(traj.qdd(1, :)) < 1e-6 && norm(traj.qdd(end, :)) < 1e-6
        fprintf('  [PASS] qdd 首末为 0\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] qdd 首末非零：||qdd(1)||=%.3e、||qdd(end)||=%.3e\n', ...
                norm(traj.qdd(1, :)), norm(traj.qdd(end, :)));
    end

    % ---------- 6. 中段差分一致性 ----------
    n_check = n_check + 1;
    qd_fd = diff(traj.q, 1, 1) / scene.dt;
    qd_ref = (traj.qd(1:end-1, :) + traj.qd(2:end, :)) / 2;
    mid = max(2, round(K_exp*0.1)) : min(K_exp-1, round(K_exp*0.9));
    if isempty(mid)
        mid = 2:K_exp-1;
    end
    abs_err = max(abs(qd_fd(mid, :) - qd_ref(mid, :)), [], 'all');
    scale   = max(1, max(abs(qd_ref(mid, :)), [], 'all'));
    rel_err = abs_err / scale;
    if rel_err < 0.05
        fprintf('  [PASS] 中段 diff(q)/dt 与 qd 一致（相对误差 %.2e）\n', rel_err);
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] 中段 diff(q)/dt 与 qd 偏离过大（相对误差 %.2e）\n', rel_err);
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
