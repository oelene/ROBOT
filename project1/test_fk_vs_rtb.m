function test_fk_vs_rtb(robot_type)
%TEST_FK_VS_RTB 将自编写正运动学结果与 RTB 的 fkine() 结果进行对比
%
%   test_fk_vs_rtb(robot_type)
%
%   对比内容包括：
%   1. 末端位置
%   2. 末端姿态旋转矩阵
%
%   说明：
%   如果结果不一致，可能的问题包括：
%   - 坐标系设置错误
%   - DH/MDH 参数表错误
%   - 齐次变换公式错误
%   - RTB 模型参数映射不一致

    fprintf('\n========== 自定义 FK 与 RTB 对比测试 ==========\n');

    params = robot_params(robot_type);

    % 设置一组测试关节角
    q = deg2rad([10, -20, 30, -15, 25, 5]);

    % 自编写正运动学结果
    [T_custom, ~] = forward_kinematics(q, params);

    % RTB 正运动学结果
    robot = build_rtb_robot(params);
    T_rtb_obj = robot.fkine(q);
    T_rtb = T_rtb_obj.T;

    fprintf('\n[测试关节角（弧度）]\n');
    disp(q);

    fprintf('\n[自编写 FK 结果]\n');
    disp(T_custom);

    fprintf('\n[RTB fkine 结果]\n');
    disp(T_rtb);

    % 位置误差
    pos_err = norm(T_custom(1:3, 4) - T_rtb(1:3, 4));

    % 姿态误差
    rot_err = norm(T_custom(1:3, 1:3) - T_rtb(1:3, 1:3), 'fro');

    fprintf('\n位置误差范数 = %.6f\n', pos_err);
    fprintf('姿态误差 Frobenius 范数 = %.6f\n', rot_err);

    % 可选：可视化显示 RTB 模型
    figure('Name', ['RTB 机械臂模型 - ', params.name]);
    robot.plot(q);


    fprintf('========== 对比测试结束 ==========\n');
end