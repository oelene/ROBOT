function path = plan_path(scene, params)
%PLAN_PATH 在给定场景下生成一条由起点到终点的关节空间路径
%
%   path = plan_path(scene, params)
%
%   输入：
%       scene  : 场景结构体（见 scene_easy.m / scene_hard.m）
%                必须使用的字段：
%                  scene.q_start  1×n 起始关节角
%                  scene.q_goal   1×n 目标关节角
%                  scene.obstacles M×4 球形障碍 [cx, cy, cz, r]
%       params : 由 robot_params() 返回的机械臂参数结构体
%
%   输出：
%       path   : M×n 关节空间路点矩阵
%                - 必须满足 path(1,  :) == scene.q_start
%                          path(end,:) == scene.q_goal
%                - 中间行数 M ≥ 2 由所选算法决定
%                - 每个路点须在 params.qlim 内
%                - 相邻路点连线在任务空间下不应穿过 scene.obstacles
%
%   说明：
%   本文件为开放性任务。算法可由小组自行选定（直线插值、Bezier、
%   APF、PRM、RRT 等），实现后请在课程设计报告中说明：
%       1. 所选算法名称与适用场景；
%       2. 关键参数（步长、迭代上限、采样数 等）；
%       3. 在 scene_easy / scene_hard 下的运行结果与失败情形。

    % =====================================================================
    % TODO: 实现你们组的路径规划算法
    % ---------------------------------------------------------------------
    %   要求：
    %     - 输出 path 必须为 M×n 数值矩阵（n = params.n）
    %     - 首行为 scene.q_start，末行为 scene.q_goal
    %     - 中间点须满足 qlim 约束与避障约束
    %
    %   未填时返回 NaN 矩阵，下游测试会精确报错指向本 TODO。
    % =====================================================================
    path = NaN(2, params.n);   % TODO：替换为路径规划结果

    if any(~isfinite(path(:)))
        error('路径规划失败：plan_path.m 中的 TODO 尚未完成。');
    end
end
