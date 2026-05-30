function test_pose_conversion(robot_type)
%TEST_POSE_CONVERSION 测试位姿表示方式之间的转换
%
%   test_pose_conversion(robot_type)
%
%   本测试用于帮助学生理解：
%   - 齐次变换矩阵
%   - 旋转矩阵
%   - RPY 角
%   - 欧拉角
%   之间的关系

    fprintf('\n========== 位姿表示转换测试 ==========\n');

    params = robot_params(robot_type);

    q = deg2rad([15, -10, 20, 30, -25, 40]);
    [T_end, ~] = forward_kinematics(q, params);

    [rpy_deg, eul_deg] = pose_to_rpy_eul(T_end);

    fprintf('\n末端齐次变换矩阵 T = \n');
    disp(T_end);

    fprintf('RPY 角（度） = \n');
    disp(rpy_deg);

    fprintf('欧拉角（度） = \n');
    disp(eul_deg);

    fprintf('========== 位姿转换测试结束 ==========\n');
end