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
%   本函数采用改进 DH（Modified DH / MDH）：
%       A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
%   必须保证报告中的公式推导与这里的代码表达一致。

    % 先缓存三角函数值，避免矩阵中反复调用 cos/sin。
    % ct、st、ca、sa 分别是 cos(theta)、sin(theta)、
    % cos(alpha)、sin(alpha) 的缩写。
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    % 下面的矩阵由四个基本变换按固定顺序相乘得到：
    %   Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
    %
    % 左上角 3×3 是坐标系方向的旋转矩阵；
    % 前三行第 4 列是新坐标系原点相对旧坐标系的位置；
    % 最后一行 [0 0 0 1] 用来把旋转和平移统一到齐次坐标中。
    %
    % 注意：矩阵乘法一般不满足交换律，因此不能随意调换四个变换顺序。
    T = [ ct,    -st,     0,     a;
          st*ca,  ct*ca, -sa,   -d*sa;
          st*sa,  ct*sa,  ca,    d*ca;
          0,      0,      0,     1   ];

    % 如果学生采用的是标准 DH，请在此处修改，
    % 并在课程设计报告中明确说明。
end
