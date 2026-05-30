function test_ik_roundtrip(robot_type)
%TEST_IK_ROUNDTRIP 解析法 IK 自洽性测试 (q → FK → IK → q')
%
%   test_ik_roundtrip(robot_type)
%
%   测试思路：
%   1. 任取若干组关节角 q
%   2. 用 forward_kinematics 得到目标位姿 T = FK(q)
%   3. 调用 inverse_kinematics_analytical 得到解集 Q_all (≤ 8 行)
%   4. 验证：
%        a) 解集中每一组解 q' 都满足 FK(q') ≈ T
%           (位置误差 < 1e-4，姿态 Frobenius 误差 < 1e-4)
%        b) 解集中存在与原始 q 接近的一组 (距离 < 1e-3，
%           wrap 到 (−π, π] 后比较)

    fprintf('\n========== 解析法 IK 自洽测试 ==========\n');

    params = robot_params(robot_type);

    % --- 测试关节角集合 (单位：度) ---
    % 选取依据：
    %   1. 全部位于 CR7 ∩ SR3 关节交集 + 5° 安全余量：
    %        J1, J4, J5, J6 ∈ (−170°, 170°)
    %        J2 ∈ (−150°, 135°)   (受 SR3 限制)
    %        J3 ∈ (−170°, 130°)   (受 SR3 限制)
    %   2. 全部 |q5| ≥ 30°，避开腕部奇异 (q5 ≈ 0 时 th4+th6 不唯一)
    %   3. 各轴混合正负方向，覆盖肩部、肘部、腕部多解情形
    q_list = deg2rad([
         30  -40   50   20   45   10;
         60   30  -50  -30   60   90;
        -45   60   40   60  -50  -60;
        120  -90   90  100   30  120;
        -90   45  -45  -90   60   45
    ]);

    n_test = size(q_list, 1);
    pass_count = 0;

    for i = 1:n_test
        q = q_list(i, :);
        T_target = forward_kinematics(q, params);

        try
            Q_all = inverse_kinematics_analytical(T_target, params);
        catch ME
            fprintf('[用例 %d] 解析 IK 抛出错误：\n   %s\n', i, ME.message);
            fprintf('         请根据上方 TODO 编号定位并补全代码。[FAIL]\n');
            continue;
        end

        if isempty(Q_all)
            fprintf('[用例 %d] 解析 IK 返回空解。可能原因：\n', i);
            fprintf('         (a) cos_phi 公式错误导致全部分支被跳过；\n');
            fprintf('         (b) 腕中心计算错误使目标超出可达空间。 [FAIL]\n');
            continue;
        end

        % --- 1) 每组解都应满足 FK(q') ≈ T ---
        max_pos_err = 0;
        max_rot_err = 0;
        for j = 1:size(Q_all, 1)
            T_chk = forward_kinematics(Q_all(j, :), params);
            pos_err = norm(T_chk(1:3, 4) - T_target(1:3, 4));
            rot_err = norm(T_chk(1:3, 1:3) - T_target(1:3, 1:3), 'fro');
            max_pos_err = max(max_pos_err, pos_err);
            max_rot_err = max(max_rot_err, rot_err);
        end

        % --- 2) 解集是否覆盖原始 q ---
        diffs = wrap_to_pi(Q_all - q);
        dist_to_q = vecnorm(diffs, 2, 2);
        min_dist = min(dist_to_q);

        ok = (max_pos_err < 1e-4) && (max_rot_err < 1e-4) && (min_dist < 1e-3);

        fprintf(['[用例 %d] 解数 = %d, 最大位置误差 = %.2e, ', ...
                 '最大姿态误差 = %.2e, 与原 q 最近距离 = %.2e   %s\n'], ...
                i, size(Q_all,1), max_pos_err, max_rot_err, min_dist, tag(ok));

        if ~ok
            % 给出针对性诊断
            if max_pos_err >= 1e-4
                fprintf(['         ↳ 位置误差大：检查 TODO 1 (Pw)、', ...
                         'TODO 3 (r/s)、TODO 4 (cos_phi)、TODO 5 (alpha/beta)\n']);
            end
            if max_rot_err >= 1e-4
                fprintf(['         ↳ 姿态误差大：检查 TODO 6 (R36)、', ...
                         'TODO 7 (theta4/5/6)\n']);
            end
            if max_pos_err < 1e-4 && max_rot_err < 1e-4 && min_dist >= 1e-3
                fprintf(['         ↳ FK 一致但解集不含原 q：', ...
                         '检查 TODO 2 (theta1 两组解) 是否漏写一组\n']);
            end
        end

        if ok
            pass_count = pass_count + 1;
        end
    end

    fprintf('\n通过用例数 %d / %d\n', pass_count, n_test);
    fprintf('========== 自洽测试结束 ==========\n');
end


function s = tag(ok)
    if ok
        s = '[PASS]';
    else
        s = '[FAIL]';
    end
end


function w = wrap_to_pi(x)
%WRAP_TO_PI 把角度差折算到 [-π, π]，处理多解时 q 与 q+2π 的等价性
    w = mod(x + pi, 2*pi) - pi;
end
