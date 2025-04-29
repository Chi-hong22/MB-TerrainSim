%% update_pdc_files - PDC点云文件视点信息更新工具
%
% 功能描述：
%   批量更新PDC点云文件中的视点信息，用于多波束数据后处理
%
% 作者信息：
%   作者：李琦，Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.0
%   创建日期：241225
%   最后修改：241225
%
% 版本历史：
%   v1.0 (241225) - 首次发布
%       + 实现PDC文件批量更新功能
%       + 添加文件排序处理
%       + 支持视点参数自动更新
%
% 输入参数：
%   input_folder  - [string] 输入文件夹路径
%                   包含原始PDC文件
%   output_folder - [string] 输出文件夹路径
%                   用于保存更新后的PDC文件
%   KEY_FRAME     - [Nx5] 关键帧信息矩阵
%                   [索引,x,y,z,方向角]
%   path_ins      - [Nx4] 位置和方向信息矩阵
%                   [x,y,z,phi], phi为弧度制
%
% 输出参数：
%   无直接返回值，更新后的文件保存到output_folder
%
% 注意事项：
%   1. 文件命名：PDC文件必须遵循'submap_X_frame.pdc'格式
%   2. 数据格式：path_ins数据需包含完整的位姿信息
%   3. 权限要求：确保对输出文件夹有写入权限
%
% 调用示例：
%   % 基础调用方式
%   update_pdc_files('./input/', './output/', key_frames, path_data);
%
% 依赖工具箱：
%   - 无特殊依赖
%
% 参见函数：
%   Phi2Quaternion, fileread, fopen

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


