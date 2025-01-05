% Phi2Quaternion - 欧拉角转四元数函数
% 日期：241221
% 作者：李琦
% 版本：v1.0
% 功能：将绕Z轴的旋转角度转换为四元数表示
% 参数：
%   phi - 绕Z轴的旋转角度（弧度）
% 返回值：
%   q - 四元数 [w x y z]，其中x=y=0（仅绕Z轴旋转）

function q = Phi2Quaternion(phi)
    % 计算四元数分量
    q0 = cos(phi / 2);
    q1 = 0;
    q2 = 0;
    q3 = sin(phi / 2);
    
    % 返回四元数
    q = [q0, q1, q2, q3];
end

