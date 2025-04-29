%% submap2PCD - 子地图数据格式转换工具
%
% 功能描述：
%   读取指定目录下的所有子地图txt文件，提取关键帧信息和点云数据，
%   将其转换为bathymetric_slam所需的标准PCD格式，并保存为pdc文件。
%   支持并行处理以提高效率。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn  
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：250110
%   最后修改：250110
%
% 版本历史：
%   v1.0 (241228) - 首次发布
%       + 实现基础的子地图txt到PCD格式的转换
%       + 支持批量文件并行处理
%       + 添加基本的错误处理
%       + 完成四元数转换和PCD文件生成
%   v1.1 (250110) - 修改四元数函数优化项目注释
%
% 输入参数：
%   inputDir_TXT  - [string] 输入txt文件目录路径
%                   必选参数，需包含sub_map_*.txt文件
%   outputDir_PCD - [string] 输出PCD文件目录路径
%                   必选参数，需确保目录存在且可写
%
% 输入文件格式（sub_map_*.txt）：
%   第1行：关键帧索引
%   第2行：关键帧位置 [x y z] 
%   第3行：速度方向角phi（角度制）
%   第4行及以后：点云数据 [x y z]，每行一个测深点坐标
%
% 输出文件格式（submap_*_frame.pdc）：
%   - PCD文件头：
%     # .PCD v0.7 - Point Cloud Data file format
%     VERSION 0.7
%     FIELDS x y z           - 点云字段定义
%     SIZE 4 4 4            - 每个字段占用字节数
%     TYPE F F F            - 数据类型(F表示float)
%     COUNT 1 1 1           - 每个字段的元素个数
%     WIDTH [点数]          - 点云宽度
%     HEIGHT 1              - 点云高度(1表示无序点云)
%     VIEWPOINT [x y z qw qx qy qz]  - 视点信息(关键帧位置和姿态四元数)
%     POINTS [总点数]        - 点云总点数
%     DATA ascii            - 数据存储格式
%   - 数据部分：
%     每行包含一个点的x y z坐标，以空格分隔
%
% 注意事项：
%   1. 内存要求：建议可用内存 > 单个文件大小 * 并行数
%   2. 目录权限：确保有读写权限
%   3. 文件命名：输入文件必须符合'sub_map_*.txt'格式
%   4. 处理时间：与文件数量和点云大小成正比
%
% 调用示例：
%   % 基本调用
%   submap2PCD('input_dir', 'output_dir');
%
% 依赖函数：
%   - angle2Quaternion：角度转四元数
%   - parpool：并行处理
%   - dlmread：文本文件读取
%   - fullfile：路径拼接
%
% 参见函数：
%   angle2Quaternion, parpool, dlmread, fullfile

function submap2PCD(inputDir_TXT, outputDir_PCD)
   % 获取文件列表
   files = dir(fullfile(inputDir_TXT, 'sub_map_*.txt'));
   assert(~isempty(files), '未找到sub_map_*.txt文件');

   % 文件排序
   [~, order] = sort(arrayfun(@(x) sscanf(x.name, 'sub_map_%d.txt'), files));
   files = files(order);

   % 创建并行池
   if isempty(gcp('nocreate'))
       parpool('Processes');
   end

   % 并行处理文件
   parfor fileIdx = 1:length(files)
       file = files(fileIdx);
       processSingleFile(inputDir_TXT, outputDir_PCD, file, fileIdx);
   end
end

function processSingleFile(inputDir_TXT, outputDir_PCD, file, fileIdx)
   % 读取数据
   filePath = fullfile(inputDir_TXT, file.name);
   data = dlmread(filePath);

   % 提取数据
   keyframe_index = data(1, :);
   keyframe_position = data(2, :);
   phi = data(3, 1);
   points = data(4:end, :);
   num_points = size(points, 1);

   % 计算四元数
   q = angle2Quaternion(phi);

   % 生成PCD头部
   pcdHeader = generatePCDHeader(keyframe_position, q, num_points);

   % 写入文件
   outputFilePath = fullfile(outputDir_PCD, sprintf('submap_%d_frame.pdc', fileIdx));
   writeDataToPCD(outputFilePath, pcdHeader, points);
end

function header = generatePCDHeader(position, q, num_points)
   % 生成标准PCD文件头
   header = sprintf(['# .PCD v0.7 - Point Cloud Data file format\n' ...
       'VERSION 0.7\n' ...
       'FIELDS x y z\n' ...
       'SIZE 4 4 4\n' ...
       'TYPE F F F\n' ...
       'COUNT 1 1 1\n' ...
       'WIDTH %d\n' ...
       'HEIGHT 1\n' ...
       'VIEWPOINT %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' ...
       'POINTS %d\n' ...
       'DATA ascii\n'], ...
       num_points, position(1), position(2), position(3), ...
       q(1), q(2), q(3), q(4), num_points);
end

function writeDataToPCD(filepath, header, points)
   % 写入PCD文件
   fid = fopen(filepath, 'w');
   fprintf(fid, '%s', header);
   fclose(fid);
   dlmwrite(filepath, points, '-append', 'delimiter', ' ', 'precision', 7);
end