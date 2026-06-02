function visualize_extra_path_cases(robot_type, case_names, sim_mode)
%VISUALIZE_EXTRA_PATH_CASES 额外路径规划动画展示
%
%   visualize_extra_path_cases()
%   visualize_extra_path_cases('SR3')
%   visualize_extra_path_cases('SR3', {'single_sphere_sweep_center', 'three_spheres_sweep'})
%
%   默认展示两个绕障动作：
%     1. single_sphere_sweep_center：单球阻挡，机械臂绕开中间障碍；
%     2. three_spheres_sweep：三球阻挡，机械臂生成多路点绕障路径。

    if nargin < 1 || isempty(robot_type)
        robot_type = 'SR3';
    end
    if nargin < 2 || isempty(case_names)
        case_names = {'single_sphere_sweep_center', 'three_spheres_sweep'};
    end
    if nargin < 3 || isempty(sim_mode)
        sim_mode = 'kinematic';
    end
    if ischar(case_names) || isstring(case_names)
        case_names = cellstr(case_names);
    end

    params = robot_params(robot_type);
    scenes = extra_path_scenes(params);

    fprintf('\n========== 额外绕障动画展示（%s / %s） ==========\n', robot_type, sim_mode);

    for i = 1:numel(case_names)
        idx = find(strcmp({scenes.name}, case_names{i}), 1);
        if isempty(idx)
            fprintf('[SKIP] 未找到额外场景：%s\n', case_names{i});
            continue;
        end

        scene = scenes(idx);
        fprintf('\n[%d/%d] 可视化场景：%s\n', i, numel(case_names), scene.name);

        path = plan_path(scene, params);
        traj = generate_trajectory(path, scene, params);

        fprintf('  path 路点数：%d，轨迹采样点：%d\n', size(path, 1), numel(traj.t));
        simulate_tracking(traj, scene, params, sim_mode);

        title(sprintf('%s - %s', robot_type, scene.name), 'Interpreter', 'none');
        drawnow;
    end
end
