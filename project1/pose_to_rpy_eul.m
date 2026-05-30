function [rpy_deg, eul_deg] = pose_to_rpy_eul(T)
%POSE_TO_RPY_EUL 将齐次变换矩阵转换为 RPY 角和欧拉角
%
%   [rpy_deg, eul_deg] = pose_to_rpy_eul(T)
%
%   输入：
%       T        : 4×4 齐次变换矩阵
%
%   输出：
%       rpy_deg  : 1×3，滚转-俯仰-偏航角，单位为度
%       eul_deg  : 1×3，欧拉角，单位为度
%
%   说明：
%   本函数用于帮助学生理解不同位姿表示方式之间的关系。

    R = T(1:3, 1:3);

    % 调用 RTB 中的位姿转换函数
    rpy_rad = tr2rpy(R);
    eul_rad = tr2eul(R);

    rpy_deg = rad2deg(rpy_rad);
    eul_deg = rad2deg(eul_rad);
end