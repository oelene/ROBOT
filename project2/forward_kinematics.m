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

    % 第一步：把真实关节角 q 加上各轴 offset，整理成完整 MDH 参数表。
    mdh_table = build_mdh_table(q, params);

    % T 始终表示“基坐标系 {0} 到当前坐标系”的累计变换。
    % 循环开始前还没有经过任何连杆，所以 T 是 4×4 单位矩阵。
    T = eye(4);

    % T_all(:,:,k) 用来保存每一级累计变换：
    %   T_all(:,:,1)   = ^0T_0 = I
    %   T_all(:,:,2)   = ^0T_1
    %   ...
    %   T_all(:,:,n+1) = ^0T_n
    % 保存中间坐标系是因为雅可比和全臂碰撞检测都要使用各关节位置。
    T_all = zeros(4, 4, params.n + 1);
    T_all(:, :, 1) = T;

    for i = 1:params.n
        % 读取第 i 节的四个 MDH 参数。
        a_i     = mdh_table(i, 1);
        alpha_i = mdh_table(i, 2);
        d_i     = mdh_table(i, 3);
        theta_i = mdh_table(i, 4);

        % 计算当前连杆的单步变换矩阵
        A_i = mdh_transform(a_i, alpha_i, d_i, theta_i);

        % 坐标变换链采用右乘：
        %   ^0T_i = ^0T_{i-1} * ^{i-1}T_i
        % 乘法顺序写反会变成完全不同的坐标含义。
        T = T * A_i;
        T_all(:, :, i + 1) = T;
    end

    % 循环结束后的累计矩阵就是基坐标系到末端坐标系的变换。
    % T_end(1:3,1:3) 是末端姿态，T_end(1:3,4) 是末端位置。
    T_end = T;
end
