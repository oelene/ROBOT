function mdh_table = build_mdh_table(q, params)
%BUILD_MDH_TABLE 根据关节变量和机械臂参数生成 MDH 参数表
%
%   mdh_table = build_mdh_table(q, params)
%
%   输入：
%       q         : 1xn 关节变量向量，单位为弧度
%       params    : 由 robot_params() 返回的机械臂参数结构体
%
%   输出：
%       mdh_table : n×4 的参数表，每一行为
%                   [a_{i-1}, alpha_{i-1}, d_i, theta_i]
%
%   说明：
%   1. 本模板默认所有关节均为转动关节。
%   2. 如果机械臂中包含移动关节，可在此基础上自行扩展。
%   3. 本项目采用改进 DH（Modified DH / MDH），与 RTB 中
%      Link(..., 'modified') 的参数顺序保持一致。
%   4. 建议后续项目继续保持本函数接口不变。

    q = q(:)';  % 保证 q 为行向量

    if length(q) ~= params.n
        error('输入的关节变量维度与机械臂自由度不一致。');
    end

    mdh_table = zeros(params.n, 4);

    for i = 1:params.n
        a_i     = params.a(i);
        alpha_i = params.alpha(i);
        d_i     = params.d(i);

        % MDH 转动关节：theta_i = q_i + offset_i。
        % offset_i 表示电机零位到建模零位的角度修正，不是固定姿态角。
        theta_i = q(i) + params.offset(i);

        mdh_table(i, :) = [a_i, alpha_i, d_i, theta_i];
    end
end
