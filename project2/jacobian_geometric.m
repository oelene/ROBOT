function J = jacobian_geometric(q, params)
%JACOBIAN_GEOMETRIC 计算机械臂在关节角 q 处的 6×n 几何雅可比矩阵
%
%   J = jacobian_geometric(q, params)
%
%   输入：
%       q      : 1×n 关节变量向量 (单位：弧度)
%       params : 由 robot_params() 返回的机械臂参数结构体
%
%   输出：
%       J      : 6×n 几何雅可比矩阵
%                前 3 行为线速度部分 Jv (米/弧度，或与 a, d 相同长度单位)
%                后 3 行为角速度部分 Jw (无量纲，即 1/弧度)
%
%   公式 (转动关节)：
%       第 i 列：
%           Jv_i = z_{i−1} × (p_n − p_{i−1})
%           Jw_i = z_{i−1}
%
%       其中：
%           z_{i−1} : 第 i−1 个连杆坐标系 z 轴在基系下的方向
%                     (即 T_all(:,:,i) 第 3 列)
%           p_{i−1} : 第 i−1 个连杆坐标系原点在基系下的位置
%                     (即 T_all(:,:,i) 第 4 列)
%           p_n     : 末端坐标系原点在基系下的位置
%                     (即 T_all(:,:,n+1) 第 4 列)
%
%   说明：
%       T_all(:,:,1)   = eye(4)              (基坐标系)
%       T_all(:,:,i+1) = base→关节 i 末端的累计变换
%
%   学生需要完成 2 处【填空】：写出 Jv、Jw 的表达式。

    % 通过正运动学得到末端位置。
    % 注意：本项目采用 MDH，关节 i 的转轴是执行
    % Rx(alpha_{i-1}) * Tx(a_{i-1}) 之后、Rz(theta_i) 之前的 z 轴，
    % 不能直接使用标准 DH 中的 z_{i-1}。
    T_end = forward_kinematics(q, params);

    n = params.n;
    p_end = T_end(1:3, 4);   % 末端位置

    J = zeros(6, n);
    mdh_table = build_mdh_table(q, params);
    T = eye(4);

    for i = 1:n
        a_i     = mdh_table(i, 1);
        alpha_i = mdh_table(i, 2);
        d_i     = mdh_table(i, 3);
        theta_i = mdh_table(i, 4);

        T_pre = T * rotx4(alpha_i) * transl4(a_i, 0, 0);
        z_axis = T_pre(1:3, 3);
        p_axis = T_pre(1:3, 4);

        % -----------------------------------------------------------------
        % 【填空 1】 第 i 列的线速度部分 Jv  (3×1)
        %   公式：Jv = z_{i−1} × (p_n − p_{i−1})
        %   提示：Matlab 内置 cross(a, b) 用于三维叉乘。
        % -----------------------------------------------------------------
        Jv = cross(z_axis, p_end - p_axis);

        % -----------------------------------------------------------------
        % 【填空 2】 第 i 列的角速度部分 Jw  (3×1)
        %   公式：Jw = z_{i−1}
        % -----------------------------------------------------------------
        Jw = z_axis;

        if any(~isfinite(Jv)) || any(~isfinite(Jw))
            error('几何雅可比失败：TODO 1 / TODO 2 尚未完成。');
        end

        J(:, i) = [Jv; Jw];

        T = T_pre * rotz4(theta_i) * transl4(0, 0, d_i);
    end
end


function T = rotx4(alpha)
    ca = cos(alpha);
    sa = sin(alpha);
    T = [1, 0, 0, 0;
         0, ca, -sa, 0;
         0, sa,  ca, 0;
         0, 0, 0, 1];
end


function T = rotz4(theta)
    ct = cos(theta);
    st = sin(theta);
    T = [ct, -st, 0, 0;
         st,  ct, 0, 0;
         0,   0,  1, 0;
         0,   0,  0, 1];
end


function T = transl4(x, y, z)
    T = [1, 0, 0, x;
         0, 1, 0, y;
         0, 0, 1, z;
         0, 0, 0, 1];
end
