%% angle2RotationMatrix - 生成绕Z轴旋转的旋转矩阵（输入为角度）
%
% 功能描述：
%   根据给定的Z轴旋转角度（单位：度），生成一个3x3的右手坐标系旋转矩阵。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.2
%   创建日期：240409
%   最后修改：250919（重命名为 angle2RotationMatrix）
%
% 输入参数：
%   phi_deg - [double] 绕Z轴的旋转角度（单位：度），逆时针为正。
%
% 输出参数：
%   R - [3x3 double] 对应的旋转矩阵。
%
% 参见：
%   world2Body, body2World

function R = angle2RotationMatrix(phi_deg)
% angle2RotationMatrix - 绕Z轴旋转(度)
    phi = deg2rad(phi_deg);
    R = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
end
