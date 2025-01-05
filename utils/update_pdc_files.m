%% PDC文件更新函数
% 该函数用于批量更新PDC点云文件中的视点信息
% 作者: 李琦，Chihong(游子昂)
% 日期: 241225
% 版本: v1.0
% 输入参数：
%   input_folder: 输入文件夹路径，包含原始PDC文件
%   output_folder: 输出文件夹路径，用于保存更新后的PDC文件
%   KEY_FRAME: 关键帧索引数组
%   path_ins: 包含位置和方向信息的路径数据
% 功能说明：
%   1. 读取输入文件夹中的所有PDC文件
%   2. 根据关键帧信息更新每个PDC文件的视点参数
%   3. 将更新后的文件保存到输出文件夹中
% 注意事项：
%   - PDC文件名格式必须为'submap_X_frame.pdc'
%   - path_ins数据格式为[x,y,z,phi]
%   - 需要确保输入参数的维度匹配

function update_pdc_files(input_folder, output_folder, KEY_FRAME, path_ins)
    % 列出输入文件夹中所有的 .pdc 文件
    files = dir(fullfile(input_folder, 'submap_*_frame.pdc'));
    
    % 提取文件名中的数字部分并按升序排序
    file_indices = arrayfun(@(f) sscanf(f.name, 'submap_%d_frame.pdc'), files);
    [~, sorted_indices] = sort(file_indices);
    files = files(sorted_indices);

    % 遍历排序后的 .pdc 文件
    for i = 1:length(files)
        % 获取当前文件的完整路径
        file_path = fullfile(files(i).folder, files(i).name);
        
        % 读取当前关键帧索引
        key_frame_index = KEY_FRAME(i);
        
        % 从 path_ins 中提取关键帧的坐标和速度方向
        key_frame_data = path_ins(key_frame_index, :);
        key_position = key_frame_data(1:3);   % 关键帧的坐标
        key_phi = key_frame_data(4);          % 关键帧的速度方向
        
        % 将速度方向角转换为四元数
        key_quaternion = Phi2Quaternion(key_phi);
        
        % 打开 .pdc 文件并读取内容
        file_content = fileread(file_path);
        file_lines = strsplit(file_content, '\n'); % 按行分割内容
        
        % 替换第9行中的 VIEWPOINT 参数
        viewpoint_line = sprintf('VIEWPOINT %.6f %.6f %.6f %.6f %.6f %.6f %.6f', ...
                                 key_position(1), key_position(2), key_position(3), ...
                                 key_quaternion(1), key_quaternion(2), key_quaternion(3), key_quaternion(4));
        file_lines{9} = viewpoint_line;
        
        % 拼接更新后的内容
        updated_content = strjoin(file_lines, '\n');
        
        % 另存为指定文件夹下的同名文件
        output_path = fullfile(output_folder, files(i).name);
        fid = fopen(output_path, 'w');
        fprintf(fid, '%s', updated_content);
        fclose(fid);
    end
end


