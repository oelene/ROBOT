function robot = build_rtb_robot(params)
%BUILD_RTB_ROBOT 使用 RTB 工具箱建立机械臂模型
%
%   robot = build_rtb_robot(params)
%
%   输入：
%       params : 机械臂参数结构体
%
%   输出：
%       robot  : RTB 中的 SerialLink 机械臂对象
%
%   说明：
%   本函数用于调用 Peter Corke Robotics Toolbox 建立机器人模型，
%   并通过 fkine() 与学生自编写的正运动学结果进行对比验证。
%
%   注意：
%   RTB 模型中使用的参数形式必须与学生自身推导采用的
%   DH/MDH 定义保持一致，否则对比结果可能不一致。

    L(1, params.n) = Link();

    for i = 1:params.n
        % TODO:
        % 根据所采用的 MDH 形式，正确配置 Link 对象。
        % 下方是转动关节的常见写法模板。
        L(i) = Link('revolute', ...
                    'a', params.a(i), ...
                    'alpha', params.alpha(i), ...
                    'd', params.d(i), ...
                    'offset', params.offset(i), ...
                    'modified');

        % 可选：设置关节范围
        L(i).qlim = params.qlim(i, :);
    end

    robot = SerialLink(L, 'name', params.name);

    % 可选：
    % 如果末端工具坐标系存在额外偏置，可在这里设置 robot.tool
    % 例如：
    % robot.tool = transl(0, 0, 0);
end