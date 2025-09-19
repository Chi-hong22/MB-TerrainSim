% visualizeSubmaps 可视化已加载的子地图集合
%
% 输入:
%   measurements (cell): 每个元胞包含 [N x 3] 点云矩阵（通常为全局坐标）
%   可选参数:
%       'SampleRate'  (0-1, 默认1.0)
%       'ColorBy'     ('z'|'submap'|'random', 默认'z')
%       'MarkerSize'  (默认1)
%       'ShowIndividual' (逻辑, 默认false)
%       'Title'       (char, 默认 'Aggregated Submaps')
%       'UseParallel' (逻辑, 默认false)
function visualizeSubmaps(measurements, varargin)
    p = inputParser;
    addRequired(p, 'measurements', @(x) iscell(x));
    addParameter(p, 'SampleRate', 1.0, @(x) isnumeric(x) && isscalar(x) && x > 0 && x <= 1);
    addParameter(p, 'ColorBy', 'z', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MarkerSize', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'ShowIndividual', false, @islogical);
    addParameter(p, 'Title', 'Aggregated Submaps', @(x) ischar(x) || isstring(x));
    addParameter(p, 'UseParallel', false, @islogical);
    parse(p, measurements, varargin{:});

    sample_rate = p.Results.SampleRate;
    color_by = char(p.Results.ColorBy);
    marker_size = p.Results.MarkerSize;
    show_individual = p.Results.ShowIndividual;
    plot_title = char(p.Results.Title);
    use_parallel = p.Results.UseParallel;

    if isempty(measurements)
        warning('measurements 为空，无法可视化'); return;
    end
    valid_measurements = measurements(~cellfun(@isempty, measurements));
    if isempty(valid_measurements)
        warning('所有子地图为空，无法可视化'); return;
    end
    num_submaps = length(valid_measurements);
    fprintf('[Viz] 可视化 %d 个子地图\n', num_submaps);

    if show_individual
        visualizeIndividualSubmaps(valid_measurements, sample_rate, marker_size);
        return;
    end

    % 聚合
    if use_parallel && num_submaps > 4
        sampled = cell(num_submaps,1); labels = cell(num_submaps,1);
        parfor i = 1:num_submaps
            pts = valid_measurements{i};
            if sample_rate < 1.0
                np = size(pts,1); ns = max(1, round(np*sample_rate));
                idx = randsample(np, ns);
                sampled{i} = pts(idx,:);
            else
                sampled{i} = pts;
            end
            if strcmpi(color_by,'submap')
                labels{i} = i*ones(size(sampled{i},1),1);
            end
        end
        all_points = vertcat(sampled{:});
        if strcmpi(color_by,'submap'), submap_labels = vertcat(labels{:}); end
    else
        all_points_cell = cell(num_submaps,1); submap_labels_cell = cell(num_submaps,1);
        for i = 1:num_submaps
            pts = valid_measurements{i};
            if sample_rate < 1.0
                np = size(pts,1); ns = max(1, round(np*sample_rate));
                idx = randsample(np, ns); pts = pts(idx,:);
            end
            all_points_cell{i} = pts;
            if strcmpi(color_by,'submap')
                submap_labels_cell{i} = i*ones(size(pts,1),1);
            end
        end
        all_points = vertcat(all_points_cell{:});
        if strcmpi(color_by,'submap'), submap_labels = vertcat(submap_labels_cell{:}); end
    end

    if isempty(all_points)
        warning('聚合点云为空'); return;
    end

    switch lower(color_by)
        case 'z'
            color_data = all_points(:,3); cmap = 'jet';
        case 'submap'
            color_data = submap_labels; cmap = 'lines';
        case 'random'
            color_data = rand(size(all_points,1),1); cmap = 'hsv';
        otherwise
            color_data = all_points(:,3); cmap = 'jet';
    end

    figure('Name','Submap Visualization','NumberTitle','off');
    pcshow(all_points, color_data, 'MarkerSize', marker_size);
    title(plot_title, 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    colormap(cmap); cb = colorbar;
    switch lower(color_by)
        case 'z', cb.Label.String = 'Z (m)';
        case 'submap', cb.Label.String = 'Submap Index';
        case 'random', cb.Label.String = 'Random';
    end
    axis equal; grid on; view(45,30);
end

function visualizeIndividualSubmaps(measurements, sample_rate, marker_size)
    n = length(measurements);
    n_cols = ceil(sqrt(n)); n_rows = ceil(n / n_cols);
    figure('Name','Individual Submaps','NumberTitle','off', 'Position', [100,100,200*n_cols,200*n_rows]);
    for i = 1:n
        subplot(n_rows, n_cols, i);
        pts = measurements{i};
        if sample_rate < 1.0 && size(pts,1) > 100
            np = size(pts,1); ns = max(10, round(np*sample_rate));
            idx = randsample(np, ns); pts = pts(idx,:);
        end
        scatter3(pts(:,1), pts(:,2), pts(:,3), marker_size, pts(:,3), 'filled');
        title(sprintf('Submap %d (%d pts)', i, size(pts,1)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on; colormap('jet'); view(45,30);
    end
    sgtitle('Individual Submap Visualization', 'FontSize', 14, 'FontWeight', 'bold');
end
