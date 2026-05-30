function params = robot_params(robot_type)
%ROBOT_PARAMS 返回所选机械臂的结构参数
%
%   params = robot_params(robot_type)
%
%   输入：
%       robot_type : 字符串或字符数组
%                    可选 'CR7' 或 'SR3'
%
%   输出：
%       params     : 机械臂参数结构体
%
%   说明：
%   本函数是整个项目统一的参数入口。
%   后续的轨迹生成、仿真测试等项目建议继续调用本函数，
%   因此不建议随意修改函数名称和输入输出接口。

    if isstring(robot_type)
        robot_type = char(robot_type);
    end
    % 根据小组选择的机器人型号填写
    switch upper(robot_type)
        case 'CR7'
            params.name = 'CR7';
            params.n = 6;

            % TODO:
            % 根据 CR7 的结构图和参数表，填写以下 MDH/几何参数。
            % ！！！！下方数值仅为占位示例，不代表真实机械臂参数。
            params.a      = [0, 300, 250, 0, 0, 0];
            params.alpha  = [pi/2, 0, 0, pi/2, -pi/2, 0];
            params.d      = [300, 0, 0, 200, 0, 80];
            params.offset = [0, 0, 0, 0, 0, 0];

            % 可选：关节范围，后续做轨迹规划和仿真时可以继续使用
            params.qlim = repmat([-pi, pi], 6, 1);

        case 'SR3'
            params.name = 'SR3';
            params.n = 6;

            % SR3 MDH 参数。
            % 每一行参数为 [a_{i-1}, alpha_{i-1}, d_i, theta_i]。
            params.a      = [0, 0, 290, 0, 0, 136];
            params.alpha  = [0, pi/2, pi, pi/2, -pi/2, pi/2];
            params.d      = [344, 0, 0, 290, 0, 103.5];
            params.offset = [pi, pi/2, pi/2, pi, 0, 0];

            params.qlim = [
                -pi, pi;
                -130*pi/180, 130*pi/180;
                -150*pi/180, 150*pi/180;
                -pi, pi;
                -120*pi/180, 120*pi/180;
                -2*pi, 2*pi
            ];
        otherwise
            error('不支持的机械臂型号，请选择 ''CR7'' 或 ''SR3''。');
    end
end
