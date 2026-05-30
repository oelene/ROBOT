clc;
clear;
close all;

% MAIN_PROJECT2_TEST
% 本文件为 Project 2 的总入口文件。
% 小组只需要修改下方的 robot_type，
% 并完成其他文件中的 TODO 内容即可。
%
% 注意：
% 本项目复用 Project 1 中的 robot_params / forward_kinematics /
% mdh_transform / build_mdh_table / build_rtb_robot 等代码，
% 因此运行前请把 Project 1 中已经填好的 .m 文件
% 与本项目所有 .m 文件放在同一个文件夹下 (Matlab 当前工作目录)。

% 选择机械臂型号：'CR7' 或 'SR3'
robot_type = 'CR7';

fprintf('=========================================\n');
fprintf(' Project 2：机械臂逆运动学求解与验证\n');
fprintf(' 当前选择机械臂型号：%s\n', robot_type);
fprintf('=========================================\n');

% 测试 1：解析法 IK 自洽测试 (q -> FK -> IK -> 检查 FK 一致)
test_ik_roundtrip(robot_type);

% 测试 2：解析法 IK 与 RTB 工具箱 ikine6s / ikunc 对比
test_ik_vs_rtb(robot_type);

% 测试 3：几何雅可比矩阵正确性测试 (与数值差分对比)
test_jacobian(robot_type);

% 测试 4 (备选)：基于雅可比的数值 IK (DLS) 收敛测试
test_ik_numerical(robot_type);

fprintf('\n全部测试完成。\n');
