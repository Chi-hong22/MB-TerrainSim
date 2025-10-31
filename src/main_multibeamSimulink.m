%% main_multibeamSimulink - 多波束声呐海底地形仿真采集工具
%
% 功能描述：
%   实现多波束声呐对海底地形的模拟采集，包括地形数据处理、AUV轨迹模拟、
%   多波束声呐仿真以及数据记录与导出功能
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：241217
%   最后修改：250429
%
% 版本历史：
%   v1.1 (250104) - 更新
%       + 新增NESP地形数据支持
%       + 优化数据存储格式
%       + 改进可视化效果
%   v1.0 (241217) - 首次发布
%       + 实现基础多波束仿真功能
%       + 支持AUV轨迹导入
%       + 添加3D可视化
%
% 输入文件：
%   - XYZ.mat 或 MapPoint_*.mat   - 地形数据文件
%   - *_path_data.mat            - AUV轨迹数据
%
% 输出文件：
%   - *_recoder.mat              - 包含AUV位姿和测深点数据的记录文件
%
% 主要参数：
%   SONAR_DEPTH    - [double] 声呐深度(m)
%   SONAR_RANGE    - [double] 声呐探测距离(m)
%   SONAR_ANGLE    - [double] 声呐扇面角度(度)
%   SONAR_BEAM_NUM - [int] 波束数量
%   TERRAIN_OFFSET - [double] 地形显示偏移量(m)
%
% 注意事项：
%   1. 需确保输入数据文件存在且格式正确
%   2. 声呐参数需根据实际应用场景调整
%   3. 建议预留足够内存空间用于数据处理
%
% 调用示例：
%   % 直接运行脚本即可
%   main_multibeamSimulink
%
% 依赖工具箱：
%   - Mapping Toolbox
%   - Statistics and Machine Learning Toolbox
%
% 参见函数：
%   multibeam, surf, pcshow

%% 数据加载
clear variables;
close all;
clc;

%% 配置文件加载
cfg = config();
fprintf('已加载配置文件 (版本: %s)\n', cfg.version);

%% 加载地形数据
% load XYZ.mat; % 牛师兄原始地形数据
load Data/MapPoint_900_900.mat; % NESP地形数据
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
load Data/250826_Processed_path_data.mat;
auv_x = processed_path(:, 1);    % AUV x坐标
auv_y = processed_path(:, 2);    % AUV y坐标
auv_heading = processed_path(:, 3);   % AUV艏向角，角度制

%% 多波束声呐参数配置（从配置文件读取）
% 从配置文件获取多波束声呐参数
SONAR_DEPTH = cfg.sonar.depth;              % 声呐深度，单位：米
SONAR_RANGE = cfg.sonar.range;              % 声呐探测距离，单位：米
SONAR_ANGLE = cfg.sonar.angle;              % 声呐扇面角度(单侧)，单位：度
SONAR_BEAM_NUM = cfg.sonar.beam_num;        % 波束数量
TERRAIN_OFFSET = cfg.sonar.terrain_offset;  % 地形偏移量，单位：米 地形下移偏移量，用于可视化区分，预估条带宽30m左右   40

% 输出参数信息
fprintf('\n多波束声呐参数配置:\n');
fprintf('  - 声呐深度: %.1f m\n', SONAR_DEPTH);
fprintf('  - 探测距离: %.1f m\n', SONAR_RANGE);
fprintf('  - 扇面角度: %.1f°\n', SONAR_ANGLE);
fprintf('  - 波束数量: %d\n', SONAR_BEAM_NUM);
fprintf('  - 地形偏移: %.1f m\n\n', TERRAIN_OFFSET);


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

% 获取当前脚本所在路径
current_script_path = fileparts(mfilename('fullpath'));
% 设置存储路径为当前脚本路径的上一级文件夹下的Data文件夹
data_path = fullfile(current_script_path, '..', 'Data');

% 如果目录不存在，则创建它
if ~exist(data_path, 'dir')
    mkdir(data_path);
end

save_date_time = datetime('now');
filename = sprintf('%02d%02d%02d_recoder.mat', ...
   mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
save(fullfile(data_path, filename), 'recoder');