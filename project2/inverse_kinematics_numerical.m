function [q, info] = inverse_kinematics_numerical(T_target, q0, params, opts)
%INVERSE_KINEMATICS_NUMERICAL 基于阻尼最小二乘法 (DLS) 的数值逆运动学
%
%   [q, info] = inverse_kinematics_numerical(T_target, q0, params, opts)
%
%   输入：
%       T_target : 4×4 末端目标位姿
%       q0       : 1×n 初始关节角 (迭代起点)
%       params   : 机械臂参数结构体
%       opts     : (可选) struct，可包含字段
%                   .max_iter  最大迭代次数 (默认 200)
%                   .tol       收敛判据    (默认 1e-6)
%                   .lambda    DLS 阻尼系数 (默认 0.01)
%                   .verbose   是否打印每步迭代误差 (默认 false)
%
%   输出：
%       q        : 1×n 收敛后的关节角
%       info     : struct，包含
%                   .iter       实际迭代次数
%                   .err_hist   每步 ||e|| 历史 (列向量)
%                   .converged  是否在 max_iter 内收敛
%
%   方法说明 (备选内容)：
%       本课程主线为解析法逆运动学；本文件提供基于 Jacobian 的数值
%       逆运动学作为对照与扩展，便于理解"逆向问题为何天然是迭代的"，
%       同时为 Project 3 的轨迹跟踪/控制做铺垫。
%
%   DLS 迭代公式：
%       Δq = J^T (J · J^T + λ²·I)^(-1) e
%       其中 e = [位置误差 ; 姿态误差]，6×1 列向量；
%            λ²·I 项保证在奇异附近 (J·J^T) 接近退化时仍可求逆。
%
%   学生需要完成 2 处【填空】。

    if nargin < 4 || isempty(opts);   opts = struct();    end
    if ~isfield(opts, 'max_iter');    opts.max_iter = 200;  end
    if ~isfield(opts, 'tol');         opts.tol      = 1e-6; end
    if ~isfield(opts, 'lambda');      opts.lambda   = 0.01; end
    if ~isfield(opts, 'verbose');     opts.verbose  = false;end

    q = q0(:)';
    err_hist = zeros(opts.max_iter, 1);
    converged = false;

    for k = 1:opts.max_iter
        % 当前末端位姿
        T_cur = forward_kinematics(q, params);

        % 6×1 位姿误差 e = [位置误差 ; 姿态误差 (轴角形式)]
        ep = T_target(1:3, 4) - T_cur(1:3, 4);
        R_err = T_target(1:3, 1:3) * T_cur(1:3, 1:3)';
        eo = rotm_to_axisangle(R_err);
        e  = [ep; eo];

        err_hist(k) = norm(e);
        if opts.verbose
            fprintf('  iter %3d  ||e|| = %.6e\n', k, err_hist(k));
        end

        if err_hist(k) < opts.tol
            converged = true;
            break;
        end

        % 当前几何雅可比 (调用学生写的 jacobian_geometric)
        J = jacobian_geometric(q, params);

        % -----------------------------------------------------------------
        % 【填空 1】 写出 DLS 增量公式
        %   Δq = J^T · (J · J^T + λ²·I)^(-1) · e
        %   提示：
        %     - I 是 6×6 单位矩阵
        %     - 用 backslash 比 inv() 数值更稳定：
        %         A \ b   等价于   inv(A) * b
        %     - 期望维度：dq 为 n×1 列向量 (这里 n = 6)
        % -----------------------------------------------------------------
        I6 = eye(6);
        dq = NaN(params.n, 1);   % TODO 1

        if any(~isfinite(dq))
            error('数值 IK 失败：TODO 1 (DLS 增量) 尚未完成。');
        end

        % -----------------------------------------------------------------
        % 【填空 2】 用 dq 更新 q
        %   提示：q 在本函数内是 1×n 行向量，dq 是 n×1 列向量，
        %         注意维度匹配 (q + dq' 或 q + reshape(dq,1,[]))。
        % -----------------------------------------------------------------
        q_new = NaN(1, params.n);   % TODO 2：写出 q + dq 的更新

        if any(~isfinite(q_new))
            error('数值 IK 失败：TODO 2 (q 更新) 尚未完成。');
        end
        q = q_new;
    end

    info.iter      = k;
    info.err_hist  = err_hist(1:k);
    info.converged = converged;
end


function w = rotm_to_axisangle(R)
%ROTM_TO_AXISANGLE 把 3×3 旋转矩阵转成 3×1 轴角向量 (axis × angle)
    cos_th = (trace(R) - 1) / 2;
    cos_th = max(-1, min(1, cos_th));
    th = acos(cos_th);
    if abs(th) < 1e-9
        w = [0; 0; 0];
    else
        w = th / (2 * sin(th)) * [R(3,2) - R(2,3);
                                  R(1,3) - R(3,1);
                                  R(2,1) - R(1,2)];
    end
end
