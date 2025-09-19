function measurements = loadAllSubmaps(pcd_folder, varargin)
% LOADALLSUBMAPS 从目录加载所有PCD/PDC子地图并可选转换到全局坐标
%
% 输入:
%   pcd_folder (string): 存放PCD/PDC文件的目录路径
%   varargin: 可选参数对，支持以下选项:
%       'MaxFiles' - 最大文件数限制 (默认: 无限制)
%       'Verbose' - 是否显示详细信息 (默认: true)
%       'UseParallel' - 是否使用并行处理 (默认: false)
%       'TransformToGlobal' - 是否转换到全局坐标系 (默认: true)
%
% 输出:
%   measurements (cell): 元胞数组，每个元胞包含一个 [N x 3] 的点云矩阵（全局或局部坐标）

    % 参数解析
    p = inputParser;
    addRequired(p, 'pcd_folder', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MaxFiles', [], @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Verbose', true, @islogical);
    addParameter(p, 'UseParallel', false, @islogical);
    addParameter(p, 'TransformToGlobal', true, @islogical);
    parse(p, pcd_folder, varargin{:});

    max_files = p.Results.MaxFiles;
    verbose = p.Results.Verbose;
    use_parallel = p.Results.UseParallel;
    transform_to_global = p.Results.TransformToGlobal;

    % 目录与文件列表
    % 确保 transform 工具在路径上
    addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'transform')));
    pcd_folder = char(pcd_folder);
    if verbose
        fprintf('[Submap] 扫描目录: %s\n', pcd_folder);
    end
    if ~exist(pcd_folder, 'dir')
        error('目录不存在: %s', pcd_folder);
    end

    pcd_files = dir(fullfile(pcd_folder, 'submap_*.pcd'));
    pdc_files = dir(fullfile(pcd_folder, 'submap_*.pdc'));
    all_files = [pcd_files; pdc_files];
    if isempty(all_files)
        warning('未找到匹配文件（submap_*.pcd / .pdc）: %s', pcd_folder);
        measurements = {};
        return;
    end

    if ~isempty(max_files) && length(all_files) > max_files
        all_files = all_files(1:max_files);
        if verbose
            fprintf('[Submap] 限制文件数: %d\n', max_files);
        end
    end

    num_files = length(all_files);
    measurements = cell(num_files, 1);
    if verbose
        fprintf('[Submap] 计划加载 %d 个文件 | 并行: %d | 转全局: %d\n', num_files, use_parallel, transform_to_global);
    end

    if use_parallel
        parfor i = 1:num_files
            file_path = fullfile(all_files(i).folder, all_files(i).name);
            try
                [pts, vp] = readSinglePcdFileWithPose(file_path);
                if transform_to_global && ~isempty(vp)
                    measurements{i} = body2World(pts, vp);
                else
                    measurements{i} = pts;
                end
            catch ME
                warning('读取失败 %s: %s', all_files(i).name, ME.message);
                measurements{i} = [];
            end
        end
    else
        for i = 1:num_files
            file_path = fullfile(all_files(i).folder, all_files(i).name);
            try
                [pts, vp] = readSinglePcdFileWithPose(file_path);
                if transform_to_global && ~isempty(vp)
                    measurements{i} = body2World(pts, vp);
                else
                    measurements{i} = pts;
                end
                if verbose && mod(i, 10) == 0
                    fprintf('[Submap] 已处理 %d/%d\n', i, num_files);
                end
            catch ME
                warning('读取失败 %s: %s', all_files(i).name, ME.message);
                measurements{i} = [];
            end
        end
    end

    % 过滤空
    empty_idx = cellfun(@isempty, measurements);
    measurements(empty_idx) = [];
    if verbose
        fprintf('[Submap] 成功加载: %d | 失败: %d\n', length(measurements), sum(empty_idx));
        if ~isempty(measurements)
            pcnts = cellfun(@(x) size(x,1), measurements);
            fprintf('[Submap] 总点数: %d | 平均每图: %.1f | min/max: %d/%d\n', sum(pcnts), mean(pcnts), min(pcnts), max(pcnts));
        end
    end
end

function [points, viewpoint] = readSinglePcdFileWithPose(file_path)
    fid = fopen(file_path, 'r');
    if fid == -1
        error('无法打开文件: %s', file_path);
    end
    points = [];
    viewpoint = struct();
    try
        header = struct(); in_header = true;
        while in_header
            line = fgetl(fid);
            if ~ischar(line)
                error('意外的文件结尾');
            end
            line = strtrim(line);
            if startsWith(line, 'FIELDS')
                header.fields = strsplit(line); header.fields = header.fields(2:end);
            elseif startsWith(line, 'POINTS')
                parts = strsplit(line); header.num_points = str2double(parts{2});
            elseif startsWith(line, 'VIEWPOINT')
                parts = strsplit(line);
                if length(parts) >= 8
                    viewpoint.position = [str2double(parts{2}), str2double(parts{3}), str2double(parts{4})];
                    % PCD VIEWPOINT: tx ty tz qw qx qy qz
                    qw = str2double(parts{5}); qx = str2double(parts{6}); qy = str2double(parts{7}); qz = str2double(parts{8});
                    % 直接按PCD格式存储，无需重排序
                    viewpoint.quaternion = [qw, qx, qy, qz];
                end
            elseif startsWith(line, 'DATA')
                parts = strsplit(line); header.data_type = parts{2}; in_header = false;
            end
        end
        % 基本校验
        req = {'x','y','z'}; idx = zeros(1,3);
        for k = 1:3
            t = find(strcmpi(header.fields, req{k}),1);
            if isempty(t), error('缺少字段: %s', req{k}); end
            idx(k) = t;
        end
        if ~strcmpi(header.data_type, 'ascii')
            error('暂不支持二进制PCD/PDC数据');
        end
        data = fscanf(fid, '%f');
        nf = numel(header.fields); mat = reshape(data, nf, [])';
        points = mat(:, idx);
        % 清理无效
        bad = any(~isfinite(points),2);
        if any(bad), points(bad,:) = []; end
    catch ME
        fclose(fid); rethrow(ME);
    end
    fclose(fid);
end
