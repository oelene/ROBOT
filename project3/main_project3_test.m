clc;
clear;
close all;

% MAIN_PROJECT3_TEST
% 本文件为 Project 3 的总入口文件。
% 小组只需要修改下方三个变量：
%   robot_type : 'CR7' 或 'SR3'（与 Project 1/2 保持一致）
%   scene_name : 'easy' 或 'hard'
%   sim_mode   : 'kinematic'（默认）或 'dynamic'
% 并完成 plan_path.m 与 generate_trajectory.m 中的 TODO。
%
% 注意：
% 本项目复用 Project 1/2 中的若干 .m 文件，运行前请把已经填好的
% Project 1/2 文件与本项目所有 .m 文件放在同一个 Matlab 当前
% 工作目录下（或一起 addpath）。

robot_type = 'SR3';
scene_name = 'easy';
sim_mode   = 'kinematic';

% 是否在基础测试结束后额外展示几个绕障动画。
show_extra_visual_cases = true;
extra_visual_cases = {'single_sphere_sweep_center', 'three_spheres_sweep'};

fprintf('=========================================\n');
fprintf(' Project 3：路径规划 + 轨迹生成 + 跟踪\n');
fprintf(' 机械臂型号：%s  场景：%s  仿真模式：%s\n', ...
        robot_type, scene_name, sim_mode);
fprintf('=========================================\n');

% 测试 1：路径规划合法性
test_path_planning(robot_type, scene_name);

% 测试 2：轨迹平滑性
test_trajectory(robot_type, scene_name);

% 测试 3：端到端跟踪（含 RTB 动画展示）
test_tracking(robot_type, scene_name, sim_mode);

% 额外可视化：展示更多路径规划/避障动作
if show_extra_visual_cases
    visualize_extra_path_cases(robot_type, extra_visual_cases, sim_mode);
end

fprintf('\n全部测试完成。\n');
