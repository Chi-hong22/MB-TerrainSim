%% 项目简介
% 日期：241219
% 作者：Chihong（游子昂）
% 版本：v1.0
% 本脚本旨在对接牛师兄多波束模拟脚本 `main_multibeamSimulink.m` 的输出数据 `recoder`，
% 实现子地图生成和处理。具体步骤包括加载数据、添加噪声、创建子地图、坐标转换、以及生成 PCD 文件。
% 最终目标是为 bathymetric_slam 提供格式化的子地图数据。
% 
% createSubmap
% 从地形数据采集处得到recoder，运行程序后得到sub_maps,一个元胞数组，每个元素代表一个子地图，直接由recoder按行切割而来。每80m或者Y变化过大划为一个子地图，当前子地图过小时划入上一个子地图。
% coordinateTransform
% 读取sub_maps，得到子地图数量的sub_map_i.txt文件，其中第一行为关键帧索引，第二行为关键帧位置，第三行为关键帧速度方向。此后的行为转换到关键帧坐标系下的测深点数据。此外KEY_FRAME存关键帧的索引以及关键帧位置以及速度方向。
% submap2PCD
% 读取指定目录下的所有sub_map_i.txt，将他们转化为bathymetric_slam需要的数据格式。其中速度方向转化为了四元数的形式。存为submap_i_frame.pdc

%% 初始化
clc;
clear;
close all;

% 获取当前工作目录
current_path = pwd;
% 添加当前目录及子目录到搜索路径 
addpath(genpath(current_path));

% 载入数据
load(fullfile('Data', '250104_recoder.mat'));        % 多波束记录数据
load(fullfile('Data', '250104_Ins_path_simulated_data.mat')); % 惯导轨迹数据
fprintf('Step 1 - 载入数据完成\n');

%% 数据预处理 - 添加误差
recoder_with_ins_error = addNoiseToRecorder(recoder, ins_path_simulated, ins_simulated_error);
fprintf('Step 2 - 误差添加完成\n');

% 可视化点云
visualizeRecoderPointCloud(recoder_with_ins_error, 'cloud');

%% 子地图生成
[~,submap_data,~] = createSubmap(recoder,recoder_with_ins_error);
fprintf('Step 3.1 - 子地图划分完成\n');

% 保存子地图数据
save_date_time = datetime('now');
submap_filename = sprintf('%02d%02d%02d_sub_maps_data.mat', ...
   mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
data_save_path = fullfile(current_path, 'Data');
save(fullfile(data_save_path, submap_filename), 'submap_data');
fprintf('Step 3.2 - 子地图数据保存完成: %s\n', submap_filename);

%% 子地图坐标系转换
% 创建保存目录
save_date_str = sprintf('%02d%02d%02d_sub_maps', ...
                        mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
submap_root_dir = fullfile(current_path, 'Data', save_date_str);
submap_txt_dir = fullfile(submap_root_dir, 'TXT_sub_maps');
submap_pcd_dir = fullfile(submap_root_dir, 'PCD_sub_maps');
% 创建目录
if ~exist(submap_txt_dir, 'dir') || ~exist(submap_pcd_dir, 'dir')
   try
       mkdir(fullfile(current_path, 'Data'));
       mkdir(submap_root_dir);
       mkdir(submap_txt_dir);
       mkdir(submap_pcd_dir);
   catch err
       error('目录创建失败：%s', err.message);
   end
end

% 坐标转换：全局坐标系转关键帧载体坐标系
key_frame_data = coordinateTransform(submap_data, submap_txt_dir);
fprintf('Step 4 - 坐标转换完成\n');

%% 生成PCD格式子地图
submap2PCD(submap_txt_dir, submap_pcd_dir)
fprintf('Step 5 - PCD格式转换完成\n');

% load KEY_FRAME.mat;
% load path_ins.mat;
% update_pdc_files('D:\code\算法—整理版\工具\submap_regular\submap',...
%                 'D:\code\算法—整理版\工具\子地图与惯导固连\惯导路径子地图' , ...
%                 KEY_FRAME, ...
%                 path_ins)
% delete_nan('D:\code\算法—整理版\工具\子地图与惯导固连\惯导路径子地图', ...
%             'D:\code\算法—整理版\工具\子地图与惯导固连\惯导路径子地图去除空值')
