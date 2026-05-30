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

    seeds = build_seed_set(params);
    opts = struct('max_iter', 120, ...
                  'tol', 1e-9, ...
                  'lambda', 0.05, ...
                  'verbose', false);

    solutions = [];
    for i = 1:size(seeds, 1)
        try
            [q_sol, info] = inverse_kinematics_numerical(T_target, seeds(i, :), params, opts);
        catch
            continue;
        end

        q_sol = wrap_to_pi(q_sol);
        if ~info.converged || ~is_valid_solution(q_sol, T_target, params)
            continue;
        end

        if isfield(params, 'qlim') && ~isempty(params.qlim)
            tol = 1e-7;
            if any(q_sol < params.qlim(:, 1)' - tol) || any(q_sol > params.qlim(:, 2)' + tol)
                continue;
            end
        end

        if isempty(solutions) || all(vecnorm(wrap_to_pi(solutions - q_sol), 2, 2) > 1e-5)
            solutions(end+1, :) = q_sol; %#ok<AGROW>
        end
    end

    Q_all = solutions;
end


function seeds = build_seed_set(params)
%BUILD_SEED_SET 构造覆盖常见肩/肘/腕构型的确定性初值集合。
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
        rng(7);
        random_seeds = zeros(30, params.n);
        for i = 1:size(random_seeds, 1)
            random_seeds(i, :) = qlim(:, 1)' + rand(1, params.n) .* ...
                                 (qlim(:, 2)' - qlim(:, 1)');
        end
        seeds = [seeds; random_seeds]; %#ok<AGROW>
    end
end


function ok = is_valid_solution(q, T_target, params)
%IS_VALID_SOLUTION 用 FK 验证候选解是否真正到达目标位姿。
    T_chk = forward_kinematics(q, params);
    pos_err = norm(T_chk(1:3, 4) - T_target(1:3, 4));
    rot_err = norm(T_chk(1:3, 1:3) - T_target(1:3, 1:3), 'fro');
    ok = pos_err < 1e-5 && rot_err < 1e-5;
end


function w = wrap_to_pi(x)
%WRAP_TO_PI 把角度折算到 [-pi, pi]。
    w = mod(x + pi, 2*pi) - pi;
end
