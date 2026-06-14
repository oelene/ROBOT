function Q_all = inverse_kinematics_analytical(T_target, params)
%INVERSE_KINEMATICS_ANALYTICAL 求解与当前 SR3 MDH 表一致的逆运动学
%
%   Q_all = inverse_kinematics_analytical(T_target, params)
%
%   当前使用的 SR3 MDH 表含 a6 与 d6 工具偏置，不再是 README 模板中
%   最简单的球腕形式。这里采用多初值 DLS 搜索，并对每个候选解做 FK
%   反校验和去重，保持本项目测试所需的多解输出接口。

    if params.n ~= 6
        error('当前 IK 求解器仅支持 6 自由度机械臂。');
    end

    % 重要说明：
    % 本文件保留了课程模板中的 “analytical” 名称和多解输出接口，
    % 但当前 SR3 完整 MDH 模型含末端 a6/d6 偏置，实际实现采用
    % “多初值 DLS 数值搜索 + FK 反校验”，并非严格闭式解析公式。
    %
    % 数值 IK 是局部迭代算法：不同初值可能收敛到不同肩/肘/腕构型，
    % 因此先构造一批覆盖不同区域的初始关节角。
    seeds = build_seed_set(params);
    opts = struct('max_iter', 120, ...
                  'tol', 1e-9, ...
                  'lambda', 0.05, ...
                  'verbose', false);

    % solutions 的每一行是一组已经通过校验且与已有结果不重复的解。
    solutions = [];
    for i = 1:size(seeds, 1)
        try
            % 从第 i 个初值独立运行一次 DLS。
            [q_sol, info] = inverse_kinematics_numerical(T_target, seeds(i, :), params, opts);
        catch
            % 某个初值失败不代表目标无解；跳过它，继续尝试其他初值。
            continue;
        end

        % 角度相差 2*pi 表示同一方向。统一折算到 [-pi,pi)，
        % 便于限位检查、解之间的距离比较和去重。
        q_sol = wrap_to_pi(q_sol);

        % 必须同时满足“迭代宣布收敛”和“FK 回代真正到达目标”。
        % 后一项可防止仅凭误差历史误收伪解。
        if ~info.converged || ~is_valid_solution(q_sol, T_target, params)
            continue;
        end

        if isfield(params, 'qlim') && ~isempty(params.qlim)
            % 数值计算在边界处可能有极小浮点误差，因此保留 1e-7 容差。
            tol = 1e-7;
            if any(q_sol < params.qlim(:, 1)' - tol) || any(q_sol > params.qlim(:, 2)' + tol)
                continue;
            end
        end

        % 逐行计算新解与已有解的周期角距离。如果所有距离都大于
        % 1e-5，才认为它是一组新的独立解。
        if isempty(solutions) || all(vecnorm(wrap_to_pi(solutions - q_sol), 2, 2) > 1e-5)
            solutions(end+1, :) = q_sol; %#ok<AGROW>
        end
    end

    Q_all = solutions;
end


function seeds = build_seed_set(params)
%BUILD_SEED_SET 构造覆盖常见肩/肘/腕构型的确定性初值集合。
    % 前几组是人工挑选的典型构型，目的是主动覆盖正负肩角、
    % 肘上/肘下和腕部正反弯等不同区域。
    base_deg = [
          0    0    0    0    0    0;
          0  -45   45    0   45    0;
          0   45  -45    0  -45    0;
         90    0    0    0   60    0;
        -90    0    0    0  -60    0;
         45  -45   45   45   45   45;
        -45   45  -45  -45  -45  -45;
        120  -90   90  100   30  120;
        -90   45  -45  -90   60   45
    ];
    seeds = deg2rad(base_deg);

    if isfield(params, 'qlim') && ~isempty(params.qlim)
        qlim = params.qlim;
        % 固定随机种子保证每次测试使用相同初值，方便复现实验结果。
        rng(7);
        random_seeds = zeros(30, params.n);
        for i = 1:size(random_seeds, 1)
            % 将 [0,1] 均匀随机数线性缩放到每个关节自己的限位区间。
            random_seeds(i, :) = qlim(:, 1)' + rand(1, params.n) .* ...
                                 (qlim(:, 2)' - qlim(:, 1)');
        end
        seeds = [seeds; random_seeds]; %#ok<AGROW>
    end
end


function ok = is_valid_solution(q, T_target, params)
%IS_VALID_SOLUTION 用 FK 验证候选解是否真正到达目标位姿。
    % 位置使用欧氏距离；姿态使用两个旋转矩阵之差的 Frobenius 范数。
    % 两种误差都小于阈值，才接受这组候选解。
    T_chk = forward_kinematics(q, params);
    pos_err = norm(T_chk(1:3, 4) - T_target(1:3, 4));
    rot_err = norm(T_chk(1:3, 1:3) - T_target(1:3, 1:3), 'fro');
    ok = pos_err < 1e-5 && rot_err < 1e-5;
end


function w = wrap_to_pi(x)
%WRAP_TO_PI 把角度折算到 [-pi, pi]。
    % 例如 190° 会折算为 -170°，二者表示同一个物理方向。
    w = mod(x + pi, 2*pi) - pi;
end
