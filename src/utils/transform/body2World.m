%% body2World - AUV体坐标系到世界坐标系的转换工具
%
% 功能描述：
%   将AUV体坐标系（局部）下的点或点云数据转换至世界坐标系（NED）。
%   支持两类位姿输入：
%     1) 视点结构体 viewpoint：包含 position[1x3] 与 quaternion[1x4]（顺序：qw qx qy qz）
%     2) 简化位姿向量 [x y z phi_deg]：仅含位置与艏向角（度），绕Z轴旋转
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
% 输入参数：
%   local_points - [Nx3 double] 体坐标系下的点云
%   pose_or_view - struct 或 [1x4] 向量
%                  struct: .position[1x3], .quaternion[1x4](qw qx qy qz)
%                  向量: [x y z phi_deg]
%
% 输出参数：
%   global_points - [Nx3 double] 世界坐标系下的点云
%
% 注意事项：
%   1. 若使用 quaternion，约定为 (qw qx qy qz) 且为右手坐标系
%   2. 若使用 yaw 角（phi_deg），仅绕Z轴旋转，采用 R' 完成体到世界变换
%   3. 与 world2Body 互为逆变换
%
% 调用示例：
%   Pb = [1 0 0; 0 1 0];  % 体坐标系点
%   view.position = [100 200 -5];
%   view.quaternion = [cosd(45) 0 0 sind(45)];  % qw qx qy qz 格式
%   Pw = body2World(Pb, view);

function global_points = body2World(local_points, pose_or_view)

    validateattributes(local_points, {'numeric'}, {'ncols',3});
    if isstruct(pose_or_view)
        assert(isfield(pose_or_view,'position') && isfield(pose_or_view,'quaternion'), 'viewpoint缺少字段');
        t = pose_or_view.position;
        q = pose_or_view.quaternion; % [qw qx qy qz] - 与PCD格式一致
        R = quaternion2RotationMatrix(q);
        global_points = (R' * local_points')' + t;
    else
        validateattributes(pose_or_view, {'numeric'}, {'vector','numel',4});
        t = pose_or_view(1:3);
        phi_deg = pose_or_view(4);
        R = angle2RotationMatrix(phi_deg);
        global_points = (R' * local_points')' + t;
    end
end
