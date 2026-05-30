function test_ik_vs_rtb(robot_type)
%TEST_IK_VS_RTB 解析法 IK 与 RTB 工具箱 IK (ikine6s / ikunc) 对比
%
%   test_ik_vs_rtb(robot_type)
%
%   测试思路：
%   1. 取若干组关节角 q，FK 得到目标位姿 T
%   2. 调用 RTB 的 ikine6s (球腕优先) 或 ikunc (通用数值) 求一组 q_rtb
%   3. 调用自编写的 inverse_kinematics_analytical 求 Q_all
%   4. 验证：
%        a) Q_all 中存在与 q_rtb 几乎一致的一组解；
%        b) FK(q_rtb) 与 FK(q_best) 的位置/姿态误差均极小。

    fprintf('\n========== 解析法 IK 与 RTB 对比测试 ==========\n');

    params = robot_params(robot_type);
    robot  = build_rtb_robot(params);

    % --- 测试关节角集合 (单位：度) ---
    % 选取依据 (与 test_ik_roundtrip 一致)：
    %   - 全部位于 CR7 ∩ SR3 关节交集内 (含 5° 安全余量)
    %   - 全部 |q5| ≥ 30°，避免腕部奇异
    q_list = deg2rad([
         30  -40   50   20   45   10;
         60   30  -50  -30   60   90;
        -45   60   40   60  -50  -60
    ]);

    n_test = size(q_list, 1);
    pass_count = 0;

    % 判断 ikine6s 是否可用 (要求球腕)
    use_ikine6s = false;
    try
        if robot.isspherical()
            use_ikine6s = true;
        end
    catch
        use_ikine6s = false;
    end

    for i = 1:n_test
        q = q_list(i, :);
        T_target = forward_kinematics(q, params);

        % --- RTB 求解 ---
        try
            if use_ikine6s
                try
                    q_rtb = robot.ikine6s(SE3(T_target));
                    rtb_method = 'ikine6s';
                catch
                    q_rtb = robot.ikunc(SE3(T_target), q);
                    rtb_method = 'ikunc';
                end
            else
                q_rtb = robot.ikunc(SE3(T_target), q);
                rtb_method = 'ikunc';
            end
        catch ME
            fprintf('[用例 %d] RTB IK 调用失败：%s\n', i, ME.message);
            continue;
        end

        % --- 自编写解析 IK ---
        try
            Q_all = inverse_kinematics_analytical(T_target, params);
        catch ME
            fprintf('[用例 %d] 自编写 IK 抛出错误：\n   %s   [FAIL]\n', i, ME.message);
            continue;
        end

        if isempty(Q_all)
            fprintf('[用例 %d] 自编写 IK 返回空解 [FAIL]\n', i);
            continue;
        end

        % 找出 Q_all 中与 q_rtb 最接近的解 (按 wrap 后欧氏距离)
        diffs = wrap_to_pi(Q_all - q_rtb);
        dists = vecnorm(diffs, 2, 2);
        [min_dist, idx] = min(dists);
        q_best = Q_all(idx, :);

        T_rtb_fk = forward_kinematics(q_rtb,  params);
        T_own_fk = forward_kinematics(q_best, params);

        rtb_pos_err = norm(T_rtb_fk(1:3,4) - T_target(1:3,4));
        own_pos_err = norm(T_own_fk(1:3,4) - T_target(1:3,4));
        rtb_rot_err = norm(T_rtb_fk(1:3,1:3) - T_target(1:3,1:3), 'fro');
        own_rot_err = norm(T_own_fk(1:3,1:3) - T_target(1:3,1:3), 'fro');

        ok = (min_dist < 1e-3) && (own_pos_err < 1e-4) && (own_rot_err < 1e-4);

        fprintf(['[用例 %d] RTB(%s) vs 解析法\n', ...
                 '   解析解集大小 = %d, 与 RTB 解最近距离 = %.2e\n', ...
                 '   位置误差 (RTB / 自编写) = %.2e / %.2e\n', ...
                 '   姿态误差 (RTB / 自编写) = %.2e / %.2e   %s\n'], ...
                i, rtb_method, size(Q_all,1), min_dist, ...
                rtb_pos_err, own_pos_err, rtb_rot_err, own_rot_err, tag(ok));

        if ok
            pass_count = pass_count + 1;
        end
    end

    fprintf('\n通过用例数 %d / %d\n', pass_count, n_test);
    fprintf('========== 对比测试结束 ==========\n');
end


function s = tag(ok)
    if ok
        s = '[PASS]';
    else
        s = '[FAIL]';
    end
end


function w = wrap_to_pi(x)
    w = mod(x + pi, 2*pi) - pi;
end
