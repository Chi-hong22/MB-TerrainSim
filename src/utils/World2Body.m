%% World2Body - 世界坐标系到AUV体坐标系的转换工具
%
% 功能描述：
%   将世界坐标系（NED）下的点或点云数据转换到AUV体坐标系，
%   支持单点转换和批量点云转换
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.0
%   创建日期：240409
%   最后修改：240409
%
% 版本历史：
%   v1.0 (240409) - 首次发布
%       + 实现基础的坐标转换功能
%       + 支持单点和点云批量转换
%       + 添加输入验证
%
% 输入参数：
%   A - [1x4 double] AUV位姿参数向量 [x y z phi]
%       x,y,z: AUV在世界坐标系中的位置
%       phi: AUV航向角（度），正北为0，顺时针为正
%
%   B - [Nx3 double] 待转换的点坐标矩阵
%       每行表示一个点的[x y z]坐标
%       支持单点[1x3]或多点[Nx3]输入
%
% 输出参数：
%   B_in_A - [Nx3 double] 转换后的点坐标矩阵
%            在AUV体坐标系下的坐标
%
% 注意事项：
%   1. 航向角输入单位为角度，函数内部会转换为弧度
%   2. 仅考虑航向角旋转，不考虑俯仰角和横滚角
%   3. 坐标系定义：
%      - 世界坐标系：NED（北东地）
%      - 体坐标系：前右下
%
% 调用示例：
%   % 示例1：单点转换
%   A = [100 200 -5 45];  % AUV在(100,200,-5)处，航向45度
%   B = [150 250 -8];     % 待转换点坐标
%   B_in_A = World2Body(A, B);
%
%   % 示例2：点云转换
%   A = [0 0 0 90];       % AUV在原点，航向90度
%   B = rand(1000,3);     % 1000个随机点
%   B_in_A = World2Body(A, B);

function B_in_A = World2Body(A, B)
    % 验证输入
    validateattributes(A, {'numeric'}, {'size',[1,4]});
    validateattributes(B, {'numeric'}, {'size',[NaN,3]});
    
    % 提取位置和角度
    A_coord = A(1:3);
    phi = deg2rad(A(4));  % 转换为弧度
    
    % 构造旋转矩阵
    R = [cos(phi), -sin(phi), 0;
         sin(phi),  cos(phi), 0;
         0,         0,        1];
    
    % 批量处理点云转换
    if size(B,1) > 1
        B_in_A = (R * (B' - A_coord'))';
    else
        B_in_A = R * (B' - A_coord');
        B_in_A = B_in_A';
    end
end
