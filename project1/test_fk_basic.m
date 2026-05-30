function test_fk_basic(robot_type)
%TEST_FK_BASIC 正运动学基础测试
%
%   test_fk_basic(robot_type)
%
%   测试内容包括：
%   1. 零位测试
%   2. 简单非零关节角测试

    fprintf('\n========== 正运动学基础测试 ==========\n');

    params = robot_params(robot_type);

    % 测试 1：零位测试
    q_zero = zeros(1, params.n);
    [T_zero, ~] = forward_kinematics(q_zero, params);

    fprintf('\n[零位测试]\n');
    disp('q = ');
    disp(q_zero);
    disp('末端位姿矩阵 T_end = ');
    disp(T_zero);

    % 测试 2：简单关节角测试
    q_test = zeros(1, params.n);
    q_test(1) = deg2rad(20);
    q_test(2) = deg2rad(-30);

    [T_test, ~] = forward_kinematics(q_test, params);

    fprintf('\n[简单关节角测试]\n');
    disp('q = ');
    disp(q_test);
    disp('末端位姿矩阵 T_end = ');
    disp(T_test);

    fprintf('========== 基础测试结束 ==========\n');
end