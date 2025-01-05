% World2Body - 坐标变换函数
% 功能：将世界坐标系下的点转换到AUV坐标系
% 作者: 李琦，Chihong(游子昂)
% 日期: 241225
% 版本: v1.0
% 参数说明：
%   A - 1x4向量，表示船体位置和姿态 [x y z phi]
%   B - 1x3向量，表示世界坐标系中待转换点的坐标 [x y z]
% 返回值：
%   B_in_A - 转换到船体坐标系下的点坐标

function B_in_A = World2Body(A, B)
    % 输入：
    % A - 1x4 向量，前三个元素是 A 的坐标，第四个元素是速度方向角 phi
    % B - 1x3 向量，表示点 B 的坐标
    
    % 提取 A 的坐标和角度
    A_coord = A(1:3);
    phi = A(4);
    
    % 将 phi 转换为弧度制（假设输入角度为度）
    
    % 构造绕 Z 轴的旋转矩阵（仅 X-Y 平面旋转）
    R = [cos(phi), -sin(phi), 0;
         sin(phi),  cos(phi), 0;
         0,        0,        1];
    
    % 计算 B 在 A 坐标系下的坐标
    B_in_A = R * (B' - A_coord');
    B_in_A = B_in_A';
end
