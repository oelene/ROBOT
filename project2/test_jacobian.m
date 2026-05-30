function test_jacobian(robot_type)
%TEST_JACOBIAN 解析雅可比与数值差分雅可比对比
%
%   test_jacobian(robot_type)
%
%   测试思路：
%   1. 任取若干组关节角 q
%   2. 调用 jacobian_geometric 计算解析雅可比 J_ana
%   3. 用中心差分法 (扰动每个关节 ε=1e-6) 计算数值雅可比 J_num
%   4. 比较 J_ana − J_num 的 Frobenius 范数

    fprintf('\n========== 几何雅可比矩阵测试 ==========\n');

    params = robot_params(robot_type);

    % --- 测试关节角集合 (单位：度) ---
    % 选取依据 (与其余测试统一)：CR7 ∩ SR3 关节交集 + 5° 安全余量，
    % 全部 |q5| ≥ 30° 远离腕部奇异。
    q_list = deg2rad([
         30  -40   50   20   45   10;
         60   30  -50  -30   60   90;
        -45   60   40   60  -50  -60
    ]);

    eps_perturb = 1e-6;
    n_test = size(q_list, 1);
    pass_count = 0;

    for i = 1:n_test
        q = q_list(i, :);

        try
            J_ana = jacobian_geometric(q, params);
        catch ME
            fprintf('[用例 %d] 解析雅可比抛出错误：\n   %s   [FAIL]\n', i, ME.message);
            continue;
        end

        J_num = numerical_jacobian(q, params, eps_perturb);

        err = norm(J_ana - J_num, 'fro');

        ok = err < 1e-4;
        fprintf('[用例 %d] q (deg) = %s\n', i, mat2str(round(rad2deg(q),1)));
        fprintf('         ||J_ana − J_num||_F = %.3e   %s\n', err, tag(ok));

        if ~ok
            % 简单分块诊断：分别看 Jv 和 Jw 的误差
            err_v = norm(J_ana(1:3,:) - J_num(1:3,:), 'fro');
            err_w = norm(J_ana(4:6,:) - J_num(4:6,:), 'fro');
            if err_v >= err_w
                fprintf('         ↳ 线速度部分误差更大，重点检查 TODO 1 (Jv = z × (p_n − p_{i−1}))\n');
            else
                fprintf('         ↳ 角速度部分误差更大，重点检查 TODO 2 (Jw = z_{i−1})\n');
            end
        end

        if ok
            pass_count = pass_count + 1;
        end
    end

    fprintf('\n通过用例数 %d / %d\n', pass_count, n_test);
    fprintf('========== 雅可比测试结束 ==========\n');
end


function J = numerical_jacobian(q, params, h)
%NUMERICAL_JACOBIAN 用中心差分法估计 6×n 几何雅可比 (位置 + 轴角)
    n = params.n;
    J = zeros(6, n);

    T0 = forward_kinematics(q, params);
    R0 = T0(1:3, 1:3);
    p0 = T0(1:3, 4);

    for i = 1:n
        qp = q; qp(i) = qp(i) + h;
        qm = q; qm(i) = qm(i) - h;

        Tp = forward_kinematics(qp, params);
        Tm = forward_kinematics(qm, params);

        % 位置部分：直接中心差分
        Jv = (Tp(1:3,4) - Tm(1:3,4)) / (2*h);

        % 姿态部分：(Tp.R * R0') 与 (Tm.R * R0') 的轴角差分
        wp = rotm_to_axisangle(Tp(1:3,1:3) * R0');
        wm = rotm_to_axisangle(Tm(1:3,1:3) * R0');
        Jw = (wp - wm) / (2*h);

        J(:, i) = [Jv; Jw];
    end
end


function w = rotm_to_axisangle(R)
    cos_th = (trace(R) - 1) / 2;
    cos_th = max(-1, min(1, cos_th));
    th = acos(cos_th);
    if abs(th) < 1e-9
        w = [0; 0; 0];
    else
        w = th / (2*sin(th)) * [R(3,2) - R(2,3);
                                R(1,3) - R(3,1);
                                R(2,1) - R(1,2)];
    end
end


function s = tag(ok)
    if ok
        s = '[PASS]';
    else
        s = '[FAIL]';
    end
end
