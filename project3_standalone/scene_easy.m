function scene = scene_easy(params)
%SCENE_EASY 简单场景：少量障碍、起终点距离适中
%
%   scene = scene_easy(params)
%
%   输入：
%       params : 机械臂参数结构体（用于 qlim 校验，可选使用）
%
%   输出：
%       scene  : 场景结构体，字段约定如下
%           .name       : 字符串
%           .q_start    : 1×n 起始关节角（弧度）
%           .q_goal     : 1×n 目标关节角（弧度）
%           .obstacles  : M×4 球形障碍 [cx, cy, cz, r]（单位与 params.a/d 一致）
%           .T_total    : 推荐的总运动时长（秒），用于 generate_trajectory
%           .dt         : 推荐的离散时间步长（秒）
%
%   说明：
%   障碍位置基于占位几何参数估算，若小组在 robot_params.m 中填入
%   真实尺寸后障碍越界，可在本文件中等比例调整。

    if nargin < 1
        params = [];
    end

    scene.name = 'easy';

    scene.q_start = [    0,        0,       0,    0,       0,    0];
    scene.q_goal  = [ pi/4,    -pi/6,    pi/6,    0,    pi/6,    0];

    % 单个球形障碍：圆心放在 q_start→q_goal 关节空间直线对应的
    % 末端轨迹中点附近（≈(481,199,87)），半径 90 mm 让直线必撞、
    % 路径规划必须绕行（单位 mm）
    scene.obstacles = [480, 200, 90, 90];

    scene.T_total = 3.0;
    scene.dt      = 0.02;
end
