function [q_best, idx] = select_ik_solution(Q_all, q_ref, params)
%SELECT_IK_SOLUTION 从多组 IK 解中按某种策略挑选一组
%
%   [q_best, idx] = select_ik_solution(Q_all, q_ref, params)
%
%   输入：
%       Q_all  : N×n，由 inverse_kinematics_analytical 返回的全部解
%       q_ref  : 1×n 参考关节角 (是否使用取决于你选择的策略)
%       params : 机械臂参数结构体 (含 qlim 等)
%
%   输出：
%       q_best : 1×n，被选中的关节解
%       idx    : q_best 在 Q_all 中的行号
%
% =========================================================================
% 本函数为开放性任务。
%
% 解析法 IK 通常返回多组解 (CR7 / SR3 这类 6R 球腕臂最多 8 组)，
% 不同应用场景对"哪一组解最合适"有不同要求。请你根据所选机械臂的
% 工作场景，自行设计选解方案，并在课程设计报告中说明：
%
%   1. 选解所依据的指标 (例如关节变化、位形约束、安全余量等等)；
%   2. 在多解情况下如何取舍；
%   3. 设计该方案的理由。
%
% 函数最低要求：
%   - 输入合法时返回一行 q_best 与 idx；
%   - Q_all 为空时报错。
% =========================================================================

    if isempty(Q_all)
        error('IK 无解：传入的 Q_all 为空，请检查目标位姿是否在工作空间内。');
    end
    q_ref = q_ref(:)';

    % 先按关节限位过滤，再从可行解中选择离参考关节角最近的一组。
    % 角度差按 2*pi 周期折算，避免 q 与 q+2*pi 被误判为距离很远。
    feasible = true(size(Q_all, 1), 1);
    if isfield(params, 'qlim') && ~isempty(params.qlim)
        tol = 1e-9;
        feasible = all(Q_all >= (params.qlim(:, 1)' - tol) & ...
                       Q_all <= (params.qlim(:, 2)' + tol), 2);
    end

    candidate_idx = find(feasible);
    if isempty(candidate_idx)
        candidate_idx = (1:size(Q_all, 1))';
    end

    diffs = wrap_to_pi(Q_all(candidate_idx, :) - q_ref);
    dists = vecnorm(diffs, 2, 2);
    [~, local_idx] = min(dists);

    idx = candidate_idx(local_idx);
    q_best = Q_all(idx, :);
end


function w = wrap_to_pi(x)
%WRAP_TO_PI 把角度差折算到 [-pi, pi]。
    w = mod(x + pi, 2*pi) - pi;
end
