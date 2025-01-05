% postprocessData - 多波束数据后处理函数
% 日期：250104
% 作者：Chihong（游子昂）
% 版本：v1.2 新增PCD格式输出支持
% 历史版本：v1.1 实现基础后处理功能
%
% 输入参数:
%   recoder - 原始多波束采集记录数据
%   insPath - INS路径数据
%   insError - INS误差数据
%
% 功能说明：
%   1. 数据预处理：添加INS误差
%   2. 点云可视化：显示采集结果
%   3. 子地图生成：调用createSubmap函数
%   4. 数据存储：生成TXT和PCD格式文件
%
% 输出说明：
%   在Data目录下生成带时间戳的子地图文件夹
%   包含TXT格式和PCD格式的子地图数据

function postprocessData(recoder, insPath, insError)
    % 数据预处理 - 添加误差
    recoder_with_ins_error = addNoiseToRecorder(recoder, insPath, insError);
    
    % 可视化点云
    visualizeRecoderPointCloud(recoder_with_ins_error, 'cloud');

    % 生成子地图
    [~,submap_data,~] = createSubmap(recoder,recoder_with_ins_error);
    
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
