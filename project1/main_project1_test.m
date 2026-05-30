clc;
clear;
close all;

% MAIN_PROJECT1_TEST
% 本文件为 Project 1 的总入口文件。
% 小组只需要修改下方的 robot_type，
% 并完成其他文件中的 TODO 内容即可。

% 选择机械臂型号：'CR7' 或 'SR3'
robot_type = 'SR3';

fprintf('=========================================\n');
fprintf(' Project 1：机械臂建模与正运动学测试\n');
fprintf(' 当前选择机械臂型号：%s\n', robot_type);
fprintf('=========================================\n');

% 运行基础正运动学测试
test_fk_basic(robot_type);

% 与 RTB 工具箱结果进行对比
test_fk_vs_rtb(robot_type);

% 测试位姿表示转换
test_pose_conversion(robot_type);

fprintf('\n全部测试完成。\n');