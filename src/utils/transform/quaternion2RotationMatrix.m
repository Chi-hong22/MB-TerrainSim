%% quaternion2RotationMatrix - 四元数(qw qx qy qz)转旋转矩阵
%
% 功能描述：
%   将一个四元数（quaternion）转换为一个3x3的旋转矩阵（Rotation Matrix）。
%   该函数遵循PCD文件格式和右手坐标系约定，四元数顺序为 [qw, qx, qy, qz]。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.2
%   创建日期：240409
%   最后修改：250919（统一PCD格式约定）
%
% 输入参数：
%   q - [1x4 double] 四元数向量，顺序为 [qw, qx, qy, qz]（与PCD VIEWPOINT一致）
%
% 输出参数：
%   R - [3x3 double] 对应的旋转矩阵
%
% 注意事项：
%   1. 输入四元数会自动进行归一化处理。
%   2. 如果四元数模长接近于零，将抛出错误。
%
% 参见：
%   body2World

function R = quaternion2RotationMatrix(q)

    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    n = sqrt(qx^2 + qy^2 + qz^2 + qw^2);
    if n > eps
        qx=qx/n; qy=qy/n; qz=qz/n; qw=qw/n;
    else
        error('无效四元数');
    end
    R = zeros(3,3);
    R(1,1) = 1 - 2*(qy^2 + qz^2);
    R(1,2) = 2*(qx*qy - qz*qw);
    R(1,3) = 2*(qx*qz + qy*qw);
    R(2,1) = 2*(qx*qy + qz*qw);
    R(2,2) = 1 - 2*(qx^2 + qz^2);
    R(2,3) = 2*(qy*qz - qx*qw);
    R(3,1) = 2*(qx*qz - qy*qw);
    R(3,2) = 2*(qy*qz + qx*qw);
    R(3,3) = 1 - 2*(qx^2 + qy^2);
end
