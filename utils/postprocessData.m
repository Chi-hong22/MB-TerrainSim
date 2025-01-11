%% postprocessData - 多波束数据后处理工具
%
% 功能描述：
%   对多波束声呐采集的原始数据进行后处理，包括INS误差添加、
%   点云可视化和子地图生成，最终输出TXT和PCD格式的数据文件
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.2
%   创建日期：240409
%   最后修改：240409
%
% 版本历史：
%   v1.2 (240409) - 功能增强
%       + 新增PCD格式输出支持
%       + 优化子地图生成算法
%   v1.1 (240408) - 完善功能
%       + 实现基础后处理功能
%       + 添加数据可视化
%
% 输入参数：
%   recoder   - [Nx(3M+5) double] 原始多波束采集记录数据
%   insPath   - [Nx3 double] INS路径数据 [x y heading]
%   insError  - [Nx3 double] INS误差数据 [error_x error_y error_heading]
%
% 输出参数：
%   无直接返回值，在Data目录下生成处理结果：
%   1. TXT格式子地图
%   2. PCD格式子地图
%   3. 可视化结果图像
%
% 注意事项：
%   1. 确保输入数据维度匹配
%   2. 需要足够的磁盘空间存储结果
%   3. 处理大量数据时注意内存使用
%
% 调用示例：
%   postprocessData(recoder, ins_path, ins_error);
%
% 依赖函数：
%   - addNoiseToRecorder
%   - createSubmap
%   - visualizeRecoderPointCloud
%   - coordinateTransform
%   - submap2PCD

function postprocessData(recoder, insPath, insError)
    % 数据预处理 - 添加误差
    recoder_with_ins_error = addNoiseToRecorder(recoder, insPath, insError);
    
    % 可视化原始点云
    visualizeRecoderPointCloud(recoder_with_ins_error, 'cloud');

    % 生成子地图
    [~,submap_data,~] = createSubmap(recoder,recoder_with_ins_error);
    
    % % 可视化所有子地图点云
    % figure('Name', '所有子地图点云可视化', 'NumberTitle', 'off');
    % hold on;
    % for i = 1:length(submap_data)
    %     current_submap = submap_data{i};
    %     visualizeRecoderPointCloud(current_submap, 'cloud');
    % end
    % title('所有子地图点云数据可视化');
    
    % 创建保存目录
    save_date_time = datetime('now');
    save_date_str = sprintf('%02d%02d%02d_sub_maps', ...
        mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
    
    current_path = pwd;
    submap_root_dir = fullfile(current_path, 'Data', save_date_str);
    submap_txt_dir = fullfile(submap_root_dir, 'TXT_sub_maps');
    submap_pcd_dir = fullfile(submap_root_dir, 'PCD_sub_maps');
    
    % 创建目录结构
    mkdir(fullfile(current_path, 'Data'));
    mkdir(submap_root_dir);
    mkdir(submap_txt_dir);
    mkdir(submap_pcd_dir);
    
    % 坐标转换和PCD生成
    key_frame_data = coordinateTransform(submap_data, submap_txt_dir);
    
    submap2PCD(submap_txt_dir, submap_pcd_dir);
end
