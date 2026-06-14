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

    % 无论调用者传入的是 1×n 行向量还是 n×1 列向量，都统一整理成
    % 1×n 行向量。这样后面使用 q(i) 时不会受输入形状影响。
    q = q(:)';

    if length(q) ~= params.n
        error('输入的关节变量维度与机械臂自由度不一致。');
    end

    % 预分配 n×4 数组。四列依次保存：
    % [连杆长度 a_{i-1}, 扭转角 alpha_{i-1}, 轴向偏距 d_i,
    %  MDH 旋转角 theta_i]。
    % 提前分配空间比在循环中逐行扩展矩阵更清晰、效率也更高。
    mdh_table = zeros(params.n, 4);

    for i = 1:params.n
        % a、alpha、d 是机器人安装完成后不再变化的几何常量。
        a_i     = params.a(i);
        alpha_i = params.alpha(i);
        d_i     = params.d(i);

        % MDH 转动关节：theta_i = q_i + offset_i。
        % offset_i 表示电机零位到建模零位的角度修正，不是固定姿态角。
        % 例如 q_i=0 只表示电机处于定义的零位，不一定表示 MDH 中
        % theta_i=0；漏加 offset 会使整条正运动学链从该关节开始出错。
        theta_i = q(i) + params.offset(i);

        % 每一行完整描述“坐标系 i-1 到坐标系 i”的一节 MDH 参数。
        mdh_table(i, :) = [a_i, alpha_i, d_i, theta_i];
    end
end
