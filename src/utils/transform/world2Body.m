%% world2Body - 世界坐标系到AUV体坐标系的转换工具
%
% 功能描述：
%   将世界坐标系（NED）下的点或点云数据转换到AUV体坐标系，
%   支持单点转换和批量点云转换，仅考虑艏向角(yaw)绕Z轴旋转。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：240409
%   最后修改：250919（迁移至 transform/ 并规范命名/文档）
%
% 版本历史：
%   v1.0 (240409) - 首次发布
%       + 实现基础的坐标转换功能
%       + 支持单点和点云批量转换
%   v1.1 (250919) - 迁移与规范
%       + 迁移至 src/utils/transform
%       + 统一小驼峰命名与文档格式
%
% 输入参数：
%   pose_world   - [1x4 double] AUV位姿参数向量 [x y z phi_deg]
%                  x,y,z: AUV在世界坐标系中的位置（米）
%                  phi_deg: AUV艏向角（角度），逆时针为正
%   points_world - [Nx3 double] 待转换的点坐标矩阵（世界系）
%
% 输出参数：
%   points_in_body - [Nx3 double] 转换后的点坐标矩阵（体坐标系：前右下/右手）
%
% 注意事项：
%   1. 仅考虑 yaw 绕Z轴旋转，未包含俯仰/横滚
%   2. 采用右手坐标系；角度为度
%   3. 与 body2World 互为逆变换
%
% 调用示例：
%   A = [100 200 -5 45];
%   Pw = [150 250 -8];
%   Pb = world2Body(A, Pw);

function points_in_body = world2Body(pose_world, points_world)

    validateattributes(pose_world, {'numeric'}, {'size',[1,4]});
    validateattributes(points_world, {'numeric'}, {'ncols',3});
    position_world = pose_world(1:3);
    phi_deg = pose_world(4);
    R = angle2RotationMatrix(phi_deg);
    points_in_body = (R * (points_world' - position_world'))';
end
