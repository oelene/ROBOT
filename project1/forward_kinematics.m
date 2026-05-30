function [T_end, T_all] = forward_kinematics(q, params)
%FORWARD_KINEMATICS 计算机械臂正运动学
%
%   [T_end, T_all] = forward_kinematics(q, params)
%
%   输入：
%       q      : 1xn 关节变量向量
%       params : 机械臂参数结构体
%
%   输出：
%       T_end  : 末端执行器相对于基坐标系的 4×4 齐次变换矩阵
%       T_all  : 4×4×(n+1) 的三维数组，保存各级累计变换矩阵
%
%   说明：
%   T_all(:,:,1) 为基坐标系变换矩阵，等于 eye(4)
%   T_all(:,:,i+1) 表示基坐标系到第 i 个关节坐标系的累计变换

    mdh_table = build_mdh_table(q, params);

    T = eye(4);
    T_all = zeros(4, 4, params.n + 1);
    T_all(:, :, 1) = T;

    for i = 1:params.n
        a_i     = mdh_table(i, 1);
        alpha_i = mdh_table(i, 2);
        d_i     = mdh_table(i, 3);
        theta_i = mdh_table(i, 4);

        % 计算当前连杆的单步变换矩阵
        A_i = mdh_transform(a_i, alpha_i, d_i, theta_i);

        % 更新累计变换矩阵
        T = T * A_i;
        T_all(:, :, i + 1) = T;
    end

    T_end = T;
end