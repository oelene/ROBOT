function T = mdh_transform(a, alpha, d, theta)
%MDH_TRANSFORM 根据单节 MDH 参数生成齐次变换矩阵
%
%   T = mdh_transform(a, alpha, d, theta)
%
%   输入：
%       a, alpha, d, theta : 一组 MDH 参数
%
%   输出：
%       T : 4×4 齐次变换矩阵
%
%   说明：
%   学生需要根据自己在报告中采用的 MDH 公式完成该矩阵。
%   必须保证报告中的公式推导与这里的代码表达一致。

    % TODO:
    % 根据课程中采用的 MDH 公式补全齐次变换矩阵。
    % 下方给出的是一种常见写法，请大家自行核对是否与自己的定义一致。

    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    T = [ ct,    -st,     0,     a;
          st*ca,  ct*ca, -sa,   -d*sa;
          st*sa,  ct*sa,  ca,    d*ca;
          0,      0,      0,     1   ];

    % 如果学生采用的是标准 DH，请在此处修改，
    % 并在课程设计报告中明确说明。
end