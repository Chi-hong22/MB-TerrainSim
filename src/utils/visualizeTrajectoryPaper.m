%% visualizeTrajectoryPaper - 论文级AUV轨迹可视化工具
%
% 功能描述：
%   读取既有AUV轨迹数据（n×3，[x,y,theta]），按论文级规范绘制路径，
%   并以统一放大倍数导出PNG与EPS文件，确保插入文档后等比缩放即可达到
%   基准物理尺寸（8.8 cm）与字号（9 pt）。
%   路径与预处理参数均从 config.m 读取，保持与主流程完全一致。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：251119
%   最后修改：260330
%
% 版本历史：
%   v1.0 (251119) - 首次发布
%       + 实现AUV路径数据加载与绘制
%       + 落地论文级等比放大规范（尺寸/字体/导出）
%       + 支持PNG(600 dpi)与EPS双格式导出
%   v1.1 (260330) - 接入 config.m 统一配置
%       + 输入路径默认值改为读取 cfg.data_path + cfg.path_file
%       + 输出目录默认值改为读取 cfg.data_path + 'trajectory_fig'
%       + 补全预处理参数：trim_start_enabled/index、short_test_enabled/range
%
% 输入参数：
%   input_mat_path - [string] 轨迹数据mat文件路径，可包含任意名称的n×3矩阵
%                     默认读取 config.m 中 cfg.data_path + cfg.path_file
%   output_dir     - [string] 导出图片目录（自动创建）
%                     默认读取 config.m 中 cfg.data_path + 'trajectory_fig'
%
% 输出参数：
%   无直接返回值，绘制效果显示在图窗中，并在output_dir生成PNG与EPS文件
%
% 注意事项：
%   1. MAT文件只需包含任意名称的n×3数值矩阵（至少前两列为x、y）
%   2. 均使用Arial字体，刻度朝外且无标题，仅显示X/Y坐标标签
%   3. 图窗与Paper尺寸完全一致，确保导出与显示同尺寸
%   4. PNG导出使用600 dpi，EPS采用向量绘制（-painters）
%   5. 预处理逻辑（降采样、头部裁剪、短时测试）与 main.m 主流程保持一致
%
% 调用示例：
%   visualizeTrajectoryPaper();                          % 完全使用 config.m 默认
%   visualizeTrajectoryPaper('Data/my_path.mat');        % 指定输入，输出用 config 默认
%   visualizeTrajectoryPaper('Data/my_path.mat', 'Outputs/path_fig');
%
% 依赖工具箱：
%   - 无特殊依赖（使用MATLAB基础绘图能力）
%
% 参见脚本：
%   src/config.m - 统一配置文件
%   src/main.m   - 主流程入口
function visualizeTrajectoryPaper(input_mat_path, output_dir)

    arguments
        input_mat_path (1, :) char = ''
        output_dir (1, :) char = ''
    end

    %% 基准参数
    BASE_WIDTH_CM = 8.8;
    BASE_HEIGHT_CM = 6.6;
    BASE_FONT_PT = 9;
    BASE_MARKER_SIZE = 30;
    BASE_MARKER_EDGE = 0.8;
    BASE_LINE_COLOR = [69, 69, 69]/255;   % 深灰 rgb(69,69,69)
    BASE_LINE_WIDTH_PT = 1.5;
    BASE_SCALE_FACTOR = 3.0;
    FONT_NAME = 'Arial';

    line_color = BASE_LINE_COLOR;
    line_width_pt = BASE_LINE_WIDTH_PT;
    scale_factor = BASE_SCALE_FACTOR;

    fig_width_cm = BASE_WIDTH_CM * scale_factor;
    fig_height_cm = BASE_HEIGHT_CM * scale_factor;
    font_size = BASE_FONT_PT * scale_factor;
    marker_size = BASE_MARKER_SIZE * scale_factor;
    marker_edge_width = BASE_MARKER_EDGE * scale_factor;
    line_width = line_width_pt * (scale_factor / 2); % 简约放大

    %% 配置加载与参数构建
    utils_dir = fileparts(mfilename('fullpath'));
    src_dir = fileparts(utils_dir);
    if exist(fullfile(src_dir, 'config.m'), 'file')
        addpath(src_dir);
    else
        warning('未在%s找到config.m，请确认项目结构。', src_dir);
    end
    cfg = config();

    % 通用设置：用 config 填充未指定的路径参数
    if isempty(input_mat_path)
        input_mat_path = fullfile(cfg.data_path, cfg.path_file);
    end
    if isempty(output_dir)
        output_dir = fullfile(cfg.data_path, 'trajectory_fig');
    end

    params.target_points = cfg.preprocess.target_points;
    params.scale_factor = cfg.preprocess.scale_factor;
    params.trim_start_enabled = cfg.preprocess.trim_start_enabled;
    params.trim_start_index = cfg.preprocess.trim_start_index;
    params.short_test_enabled = cfg.preprocess.short_test_enabled;
    params.short_test_range = cfg.preprocess.short_test_range;
    params.enable_ins_error = cfg.ins_error.enable;
    params.error.line_std = cfg.ins_error.line_std;
    params.error.turn_std = cfg.ins_error.turn_std;
    params.error.cumulative = cfg.ins_error.cumulative;
    params.error.turn_factor = cfg.ins_error.turn_factor;
    params.error.no_error_fraction = cfg.ins_error.no_error_fraction;
    params.error.window_size = cfg.ins_error.window_size;

    %% 数据加载
    if ~isfile(input_mat_path)
        error('输入文件不存在：%s', input_mat_path);
    end
    data_struct = load(input_mat_path);
    candidate = [];

    % 优先使用PathFollowing，其次自动检测第一个符合条件的数值矩阵
    if isfield(data_struct, 'PathFollowing')
        candidate = data_struct.PathFollowing;
    else
        field_names = fieldnames(data_struct);
        for i = 1:numel(field_names)
            value = data_struct.(field_names{i});
            if isnumeric(value) && ndims(value) == 2 && size(value, 2) >= 2
                candidate = value;
                break;
            end
        end
    end

    if isempty(candidate)
        error('文件 %s 中未找到至少两列的数值矩阵，请检查输入。', input_mat_path);
    end

    path_data = candidate;

    %% 路径预处理（与主流程保持一致）
    [processed_path, ~, ~] = preprocessData(path_data, params, cfg);
    auv_x = processed_path(:, 1);
    auv_y = processed_path(:, 2);

    %% Figure设置
    fig = figure('Color', [255, 255, 255]/255);  % 白色 rgb(255,255,255)
    set(fig, 'Units', 'centimeters', 'Position', [2, 2, fig_width_cm, fig_height_cm]);
    set(fig, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, fig_width_cm, fig_height_cm], ...
        'PaperSize', [fig_width_cm, fig_height_cm]);

    ax = axes(fig);
    hold(ax, 'on');
    plot(ax, auv_x, auv_y, 'Color', line_color, ...
        'LineWidth', line_width, 'LineJoin', 'round');
    start_fill_color = [230, 122, 93]/255;   % 起点填充 rgb(230,122,93)
    end_fill_color = [146, 219, 166]/255;    % 终点填充 rgb(146, 219, 166)
    scatter(ax, auv_x(1), auv_y(1), marker_size, start_fill_color, ...
        'filled', 'Marker', 'o', 'MarkerEdgeColor', line_color, 'LineWidth', marker_edge_width);
    scatter(ax, auv_x(end), auv_y(end), marker_size, end_fill_color, ...
        'filled', 'Marker', 'o', 'MarkerEdgeColor', line_color, 'LineWidth', marker_edge_width);

    % 坐标范围紧贴轨迹，仅保留极小安全余量
    pad_ratio = 0.035;
    x_min = min(auv_x);
    x_max = max(auv_x);
    y_min = min(auv_y);
    y_max = max(auv_y);
    x_range = max(x_max - x_min, eps);
    y_range = max(y_max - y_min, eps);
    max_range = max(x_range, y_range);

    x_adjust = (max_range - x_range) / 2;
    y_adjust = (max_range - y_range) / 2;

    x_limits = [x_min - x_adjust, x_max + x_adjust];
    y_limits = [y_min - y_adjust, y_max + y_adjust];

    pad = max_range * pad_ratio;
    x_limits = x_limits + [-pad, pad];
    y_limits = y_limits + [-pad, pad];

    axis(ax, 'equal');
    xlim(ax, x_limits);
    ylim(ax, y_limits);
    box(ax, 'off');
    grid(ax, 'off');
    ax.Layer = 'top';
    ax.TickDir = 'out';
    ax.FontName = FONT_NAME;
    ax.FontSize = font_size;
    ax.LineWidth = 0.6 * scale_factor;
    ax.XColor = [0, 0, 0]/255;  % 黑色 rgb(0,0,0)
    ax.YColor = [0, 0, 0]/255;  % 黑色 rgb(0,0,0)

    ax.TickLength = [0.02, 0.02];

    xlabel(ax, 'X (m)', 'FontName', FONT_NAME, 'FontSize', font_size, 'Interpreter', 'none');
    ylabel(ax, 'Y (m)', 'FontName', FONT_NAME, 'FontSize', font_size, 'Interpreter', 'none');

    % 统一边距布局
    margin_left = 0.16;
    margin_right = 0.1;
    margin_bottom = 0.18;
    margin_top = 0.1;
    ax.Units = 'normalized';
    ax.Position = [margin_left, margin_bottom, ...
        1 - margin_left - margin_right, 1 - margin_top - margin_bottom];

    %% 导出
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    timestamp = datestr(datetime('now'), 'yymmdd_HHMMSS');
    png_path = fullfile(output_dir, sprintf('path_%s.png', timestamp));
    eps_path = fullfile(output_dir, sprintf('path_%s.eps', timestamp));

    exportgraphics(fig, png_path, 'Resolution', 600, 'BackgroundColor', [255, 255, 255]/255);
    print(fig, eps_path, '-depsc', '-painters', '-r600');

    fprintf('路径可视化已导出：\n  PNG: %s\n  EPS: %s\n', png_path, eps_path);
end

