%% visualizeRecoderPointCloud - AUV多波束数据可视化工具
%
% 功能描述：
%   可视化AUV多波束点云数据及路径，支持三种可视化模式
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：240328
%   最后修改：240409
%
% 版本历史：
%   v1.0 (240328) - 首次发布
%       + 实现基础点云和路径可视化
%   v1.1 (240409) - 功能优化
%       + 优化代码结构
%       + 添加模式选择功能
%       + 完善错误处理
%
% 输入参数：
%   recoder - AUV状态及点云数据矩阵
%            第1列：帧数
%            第2-3列：AUV的xy坐标 [m]
%            第4列：艏向角 [deg]
%            第5列：标识符
%            第6列及之后：点云坐标(x1,y1,z1,...,x256,y256,z256) [m]
%   mode    - 可视化模式选择
%            'path' - 仅显示AUV路径
%            'cloud' - 仅显示点云数据
%            'both' - 同时显示路径和点云（默认）
%
% 输出参数：
%   无直接返回值，结果以图形方式显示
%
% 注意事项：
%   1. 内存要求：随点云数据量增加而增加
%   2. 数据格式：必须严格遵循指定格式
%   3. 坐标系：采用右手坐标系
%
% 调用示例：
%   % 示例1：默认显示（路径+点云）
%   visualizeRecoderPointCloud(recoder);
%
%   % 示例2：仅显示路径
%   visualizeRecoderPointCloud(recoder, 'path');
%
% 依赖工具箱：
%   - Computer Vision Toolbox
%   - Point Cloud Toolbox
%
% 参见函数：
%   pcshow, plot3, figure

function visualizeRecoderPointCloud(recoder, mode)

    %% 输入验证
    [rows, cols] = size(recoder);
    if cols ~= 773
        error('输入数据维度错误：列数应为773（5+3*256）');
    end

    % 设置默认模式
    if nargin < 2
        mode = 'both';
    end

    % 验证模式输入
    valid_modes = {'path', 'cloud', 'both'};
    if ~ismember(mode, valid_modes)
        error('无效的可视化模式，请选择: path, cloud, 或 both');
    end

    %% 创建图窗
    figure('Name', 'AUV数据可视化', 'NumberTitle', 'off');

    %% 路径可视化
    if ismember(mode, {'path', 'both'})
        % 提取AUV位置和艏向角数据
        auv_x = recoder(:, 2);
        auv_y = recoder(:, 3);
        % auv_heading = recoder(:, 4);  % 角度制
        
        % 绘制AUV路径
        if strcmp(mode, 'path')
            plot(auv_x, auv_y, 'b-', 'LineWidth', 2);
            hold on;
            % % 每隔一定间隔绘制艏向箭头
            % arrow_interval = max(1, floor(length(auv_x)/20));  % 控制箭头数量
            % for i = 1:arrow_interval:length(auv_x)
            %     heading_rad = deg2rad(auv_heading(i));
            %     arrow_length = 0.5;  % 箭头长度，可根据实际尺度调整
            %     dx = arrow_length * cos(heading_rad);
            %     dy = arrow_length * sin(heading_rad);
            %     quiver(auv_x(i), auv_y(i), dx, dy, 0, 'r', 'LineWidth', 1);
            % end
            grid on;
            axis equal;
            xlabel('X (m)');
            ylabel('Y (m)');
            title('AUV路径可视化');
        end
    end

    %% 点云可视化
    if ismember(mode, {'cloud', 'both'})
        % 提取并重组点云数据
        point_cloud_data = recoder(:, 6:end);
        num_points = size(point_cloud_data, 2) / 3;
        points_reshaped = zeros(rows * num_points, 3);
        
        % 重组点云数据
        for i = 1:num_points
            start_idx = (i-1)*3 + 1;
            points_reshaped((i-1)*rows+1:i*rows, :) = point_cloud_data(:, start_idx:start_idx+2);
        end
        
        % 创建点云对象并显示
        pc = pointCloud(points_reshaped);
        
        if strcmp(mode, 'cloud')
            pcshow(pc);
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            title('AUV多波束点云数据可视化');
        else
            % both模式：在同一图中显示路径和点云
            pcshow(pc);
            hold on;
            % 在3D空间中绘制AUV路径
            plot3(auv_x, auv_y, zeros(size(auv_x)), 'r-', 'LineWidth', 2);
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            title('AUV路径与点云数据可视化');
        end
        
        grid on;
        axis equal;
        view(3);
    end

end