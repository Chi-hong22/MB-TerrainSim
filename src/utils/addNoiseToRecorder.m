%% addNoiseToRecorder - AUV数据噪声添加工具
%
% 功能描述：
%   为AUV位姿及点云数据添加模拟噪声，用于仿真测试
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.0
%   创建日期：241228
%   最后修改：241228
%
% 版本历史：
%   v1.0 (241228) - 首次发布
%       + 实现基础噪声添加功能
%       + 支持位置和姿态噪声
%       + 添加点云坐标变换
%
% 输入参数：
%   recoder             - 原始数据记录 [NxM]
%                        第1列：帧数
%                        第2-3列：AUV的xy坐标 [m]
%                        第4列：艏向角 [deg]
%                        第5列：标识符
%                        第6列及之后：点云坐标(256点xyz) [m]
%   ins_path_simulated  - 添加噪声后的AUV位姿 [Nx3]
%                        [x,y,heading]
%   ins_simulated_error - AUV位姿噪声 [Nx3]
%                        [误差_x,误差_y,误差_heading]
%
% 输出参数：
%   recoder_INS        - 添加噪声后的数据记录
%                       格式与输入recoder相同
%
% 注意事项：
%   1. 数据维度：确保输入矩阵维度匹配
%   2. 角度单位：采用角度制
%   3. 坐标系：采用右手坐标系
%
% 调用示例：
%   % 添加模拟噪声
%   recoder_INS = addNoiseToRecorder(recoder, ins_path, ins_error);
%
% 依赖工具箱：
%   - 无特殊依赖
%
% 参见函数：
%   reshape, cosd, sind

function recoder_INS = addNoiseToRecorder(recoder, ins_path_simulated, ins_simulated_error)

    % 预分配输出矩阵空间
    recoder_INS = recoder;
    
    % 获取数据维度
    [num_frames, ~] = size(recoder);
    POINTS_PER_FRAME = 256;  % 每帧点云数量
    
    % 更新AUV位姿数据
    recoder_INS(:, 2:4) = ins_path_simulated;
    
    % 处理每帧点云数据
    for frame = 1:num_frames
        % 获取当前帧的误差数据
        dx = ins_simulated_error(frame, 1);
        dy = ins_simulated_error(frame, 2);
        dheading = ins_simulated_error(frame, 3);  % 角度制
        
        % 构建变换矩阵
        % 旋转矩阵
        R = [cosd(dheading), -sind(dheading), 0;
             sind(dheading),  cosd(dheading), 0;
             0,              0,              1];
        % 平移向量
        T = [dx; dy; 0];
        
        % 提取当前帧的点云数据
        point_data = reshape(recoder(frame, 6:end), 3, POINTS_PER_FRAME);
        
        % 应用变换
        transformed_points = R * point_data + T;
        
        % 更新点云数据
        recoder_INS(frame, 6:end) = transformed_points(:)';
    end
end