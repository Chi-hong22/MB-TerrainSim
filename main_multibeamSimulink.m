%% 项目简介
% 日期：241217
% 作者：Chihong（游子昂）
% 版本：v1.0
% 本脚本用于实现多波束声呐对海底地形的模拟采集。主要功能包括：
% 1. 地形数据处理：加载并预处理海底地形数据
% 2. AUV轨迹模拟：读取AUV运动轨迹并进行可视化
% 3. 多波束声呐仿真：配置声呐参数并执行地形扫测
% 4. 数据记录与导出：保存采集结果供后续处理
% 最终输出文件为包含AUV位姿和测深点数据的recoder文件

%% 数据加载
clear variables;
close all;
clc;

% 加载地形数据
% load XYZ.mat; % 牛师兄原始地形数据
load Data/241216_MapPoint_900_900.mat; % NESP地形数据
% 对NESP地形数据进行放缩
X = 10 * X ;
Y = 10 * Y ;

% 加载AUV轨迹数据

% 牛师兄原始路径数据
% trajectory_data = load('Data/path.mat');
% auv_x = trajectory_data.path(:, 1);    % AUV x坐标
% auv_y = trajectory_data.path(:, 2);    % AUV y坐标
% auv_heading = trajectory_data.path(:, 3);   % AUV艏向角

% NESP中沙礁数据路径
load Data/250104_Processed_path_data.mat;
auv_x = processed_path(:, 1);    % AUV x坐标
auv_y = processed_path(:, 2);    % AUV y坐标
auv_heading = processed_path(:, 3);   % AUV艏向角，角度制

%% 可调节参数配置
% 多波束声呐参数
SONAR_DEPTH = 0;        % 声呐深度，单位：米
SONAR_RANGE = 100;      % 声呐探测距离，单位：米
SONAR_ANGLE = 60;       % 声呐扇面角度(单侧)，单位：度
SONAR_BEAM_NUM = 256;   % 波束数量

% 地形调整参数
TERRAIN_OFFSET = -25;    % 地形下移偏移量，用于可视化区分，预估条带宽30m左右   40


%% 地形与轨迹可视化
figure;
% 绘制海底地形
surf(X, Y, Z - TERRAIN_OFFSET);
colormap("turbo")
shading interp
hold on;

% 绘制AUV轨迹
plot3(auv_x, auv_y, zeros(size(auv_x)), 'r-', 'LineWidth', 1.2);

% 设置图形属性
xlabel('X方向 (m)');
ylabel('Y方向 (m)');
zlabel('深度 (m)');
title('海底地形与AUV轨迹');
grid on;
hold off;

%% 多波束声呐仿真采集
% 组织AUV位姿数据 [x, y, heading]
auv_poses = [auv_x, auv_y, auv_heading];

% 执行多波束测深仿真
recoder = multibeam(X, Y, Z-TERRAIN_OFFSET, auv_poses, ...
    SONAR_DEPTH, SONAR_RANGE, SONAR_ANGLE, SONAR_BEAM_NUM);

% 提取点云数据 (跳过前5列姿态信息)
point_cloud = reshape(recoder(:,6:end)', 3, [])';

%% 点云数据可视化
figure;
pcshow(point_cloud);
title('多波束声呐采集点云');
xlabel('X方向 (m)');
ylabel('Y方向 (m)');
zlabel('深度 (m)');

%% 保存点云数据（可选）
% dlmwrite(OUTPUT_FILENAME, point_cloud, 'delimiter', ' ');

current_path = pwd;
save_date_time = datetime('now');
filename = sprintf('%02d%02d%02d_recoder.mat', ...
   mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
data_save_path = fullfile(current_path, 'Data');
save(fullfile(data_save_path, filename), 'recoder');