function test_ik_numerical(robot_type)
%TEST_IK_NUMERICAL DLS 数值 IK 收敛性测试 (备选内容)
%
%   test_ik_numerical(robot_type)
%
%   测试思路：
%   1. 取一组真值关节角 q_true，计算目标位姿 T_target
%   2. 在 q_true 上叠加 ±15° 随机扰动得到初值 q0
%   3. 调用 inverse_kinematics_numerical 迭代收敛
%   4. 验证收敛后 FK(q) 与 T_target 的位姿误差是否足够小
%
%   说明：
%   本测试为备选项目内容，重点在于验证学生写的雅可比 + DLS 公式
%   是否能让数值法收敛。解析法仍是本课程主线。

    fprintf('\n========== DLS 数值 IK 收敛测试 (备选) ==========\n');

    params = robot_params(robot_type);

    rng(42);   % 固定随机种子，便于复现

    % --- 测试关节角集合 (单位：度) ---
    % 选取依据 (与其余测试统一)：CR7 ∩ SR3 关节交集 + 5° 安全余量，
    % 全部 |q5| ≥ 30° 远离腕部奇异。
    % 注意：DLS 测试还要在真值上叠加 ±15° 扰动作为初值，
    % 因此真值本身需要再额外保留 15° 的边界余量 — 下方用例已满足。
    q_true_list = deg2rad([
         30  -40   50   20   45   10;
         60   30  -50  -30   60   90;
        -45   60   40   60  -50  -60
    ]);

    n_test = size(q_true_list, 1);
    pass_count = 0;

    for i = 1:n_test
        q_true = q_true_list(i, :);
        T_target = forward_kinematics(q_true, params);

        % 初值：真值 ± 15° 随机扰动
        q0 = q_true + deg2rad(15) * (2*rand(1, params.n) - 1);

        opts = struct('max_iter', 200, 'tol', 1e-6, 'lambda', 0.01, 'verbose', false);
        try
            [q_sol, info] = inverse_kinematics_numerical(T_target, q0, params, opts);
        catch ME
            fprintf('[用例 %d] 数值 IK 抛出错误：\n   %s   [FAIL]\n', i, ME.message);
            continue;
        end

        T_chk   = forward_kinematics(q_sol, params);
        pos_err = norm(T_chk(1:3,4) - T_target(1:3,4));
        rot_err = norm(T_chk(1:3,1:3) - T_target(1:3,1:3), 'fro');

        ok = info.converged && (pos_err < 1e-4) && (rot_err < 1e-4);

        fprintf(['[用例 %d] 迭代 %d 次, 最终 ||e|| = %.2e\n', ...
                 '         FK 位置误差 = %.2e, 姿态误差 = %.2e   %s\n'], ...
                i, info.iter, info.err_hist(end), pos_err, rot_err, tag(ok));

        if ok
            pass_count = pass_count + 1;
        end
    end

    fprintf('\n通过用例数 %d / %d\n', pass_count, n_test);
    fprintf('========== 数值 IK 测试结束 ==========\n');
end


function s = tag(ok)
    if ok
        s = '[PASS]';
    else
        s = '[FAIL]';
    end
end
