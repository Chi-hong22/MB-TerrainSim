%% angle2Quaternion - 角度制旋转角转四元数转换工具
%
% 功能描述：
%   将绕Z轴的旋转角度（角度制）转换为对应的四元数表示
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：241221
%   最后修改：241228
%
% 版本历史：
%   v1.0 (241221) - 首次发布
%       + 实现基础的Z轴旋转角到四元数的转换
%       + 由李琦编写完成
%   v1.1 (241228) - 功能优化
%       + 修改输入为角度制
%       + 优化变量命名和注释
%       + 由游子昂完善
%
% 输入参数：
%   angle_deg - [double] 绕Z轴的旋转角度（角度制）
%               必选参数，范围[0, 360]
%
% 输出参数：
%   quaternion - [1x4 double] 四元数数组 [qw qx qy qz]（与PCD VIEWPOINT格式一致）
%                qw: 实部 (cos(θ/2))
%                qx, qy: 0 (仅Z轴旋转)
%                qz: 虚部 (sin(θ/2))
%
% 注意事项：
%   1. 仅支持绕Z轴的旋转，不支持完整的3D旋转
%   2. 输入角度采用角度制（0-360度）
%   3. 输出四元数满足单位性质：w²+x²+y²+z²=1
%
% 调用示例：
%   % 示例1：旋转90度
%   q = angle2Quaternion(90);  % 返回 [0.7071, 0, 0, 0.7071]
%
%   % 示例2：旋转180度
%   q = angle2Quaternion(180); % 返回 [0, 0, 0, 1]
%
% 参见函数：
%   deg2rad, cos, sin

function quaternion = angle2Quaternion(angle_deg)
    % 角度转弧度
    angle_rad = deg2rad(angle_deg);
    
    % 计算四元数分量
    quat_w = cos(angle_rad / 2);  % 实部
    quat_x = 0;                   % X轴分量（无）
    quat_y = 0;                   % Y轴分量（无）
    quat_z = sin(angle_rad / 2);  % Z轴分量
    
    % 返回四元数数组 [qw qx qy qz]，与PCD VIEWPOINT格式一致
    quaternion = [quat_w, quat_x, quat_y, quat_z];
end