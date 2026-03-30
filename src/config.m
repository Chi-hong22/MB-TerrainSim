function cfg = config()
    % CONFIG - 多波束声呐仿真系统配置文件
    %
    % 功能描述：
    %   统一管理多波束声呐仿真系统中的所有配置参数，支持一体化处理 (main.m) 
    %   和分步处理 (dataPreproccess.m) 两种工作流。
    %
    % 作者：Chihong（游子昂）
    % 邮箱：you.ziang@hrbeu.edu.cn
    % 单位：哈尔滨工程大学
    % 版本：v1.1
    % 创建日期：2025-09-12
    %
    % 返回值：
    %   cfg - 包含所有配置参数的结构体
    %
    % 使用示例：
    %   cfg = config();
    %   target_points = cfg.preprocess.target_points;
    %
    %% ========== 版本信息 ==========
    cfg.version = 'v1.1';
    cfg.created_date = '2025-09-11';
    cfg.description = '多波束声呐仿真系统统一配置文件';
    cfg.last_updated = '2025-09-12'; % 新增子地图参数

    %% ========== 通用设置 ==========
    cfg.data_path = 'Data';                              % 数据文件路径
    cfg.terrain_file = 'MapPoint.mat';          % 地形数据文件名
    cfg.path_file = '260326_Ling_PathFollowing.mat';         % 路径数据文件名
    %% ========== 预处理参数 ==========
    cfg.preprocess.target_points = 70000;               % 降采样目标点数 (NESP: 70000, Comb(noNESP): 45000, Ling:70000)
    cfg.preprocess.scale_factor = 1;                    % 坐标系缩放系数，目标是900*900范畴  NESP_PathFollowing: 2, noNESP_PathFollowing: 6, Ling: 1

    % 数据开头裁剪设置
    cfg.preprocess.trim_start_enabled = true;           % 是否启用数据头部裁剪
    cfg.preprocess.trim_start_index = 400;             % 数据头部裁剪起始点 NESP_PathFollowing: 550, noNESP_PathFollowing: 1101, Ling:400

    % 短时测试设置
    cfg.preprocess.short_test_enabled = false;          % 是否启用短时测试模式
    cfg.preprocess.short_test_range = [cfg.preprocess.trim_start_index, 5000];      % 短时测试的数据范围 [起始行, 结束行]

    %% ========== 惯导误差模拟参数 ==========
    cfg.ins_error.enable = false;                      % INS误差模拟开关 (true: 启用误差, false: 理想路径)
    cfg.ins_error.line_std = [0.03, 0.05];             % 直线段随机误差标准差 [x方向, y方向]
    cfg.ins_error.turn_std = [0.0005, 0.02];           % 转弯段随机误差标准差 [x方向, y方向]
    cfg.ins_error.cumulative = [0.019, 0.0025];        % 累积误差系数 [x方向, y方向]
    cfg.ins_error.turn_factor = [0.01, 0.01];          % 转弯误差系数 [x方向, y方向]
    cfg.ins_error.no_error_fraction = 0.03;            % 无误差路径比例 (前3%路径不增加误差)
    cfg.ins_error.window_size = 40;                    % 艏向角平滑窗口大小

    %% ========== 多波束声呐参数 ==========
    cfg.sonar.depth = 0;                               % 声呐深度 (米)
    cfg.sonar.range = 100;                             % 最大探测距离 (米)
    cfg.sonar.angle = 60;                              % 单侧扇面角度 (度)
    cfg.sonar.beam_num = 256;                          % 波束数量
    cfg.sonar.terrain_offset = -25;                    % 地形偏移量 (米)，用于地形调整\可视化区分，预估条带宽30m左右   40

    %% ========== 子地图生成参数 ==========
    % 注意:该部分子地图划分逻辑之后会修改为定帧子地图,带有转弯部分的子地图直接去除
    cfg.submap.heading_threshold = 1.5;                % 平均艏向角变化率阈值(度/采样点), 用于区分直线和转弯  原始:0.9
    cfg.submap.frames_per_submap = 100;                % 每个子地图的目标帧数   原始:100
    cfg.submap.min_straight_frames = 50;               % 可独立生成子地图的最小直线段帧数 原始:75
    cfg.submap.window_size = 40;                       % 居中滑动窗口大小(帧), 用于检测转弯 


    %% ========== 可视化参数 ==========
    cfg.visualization.enable_global_view = true;       % 生成PCD后进行全局坐标聚合可视化
    cfg.visualization.sample_rate = 1.0;               % 可视化采样率

    %% ========== 配置验证 ==========
    % 可以在这里添加参数合理性检查
    if cfg.preprocess.target_points <= 0
        error('config: target_points must be a positive integer');
    end
    
    if cfg.preprocess.scale_factor <= 0
        error('config: scale_factor must be a positive number');
    end
    
    if cfg.preprocess.short_test_enabled && ...
       (cfg.preprocess.short_test_range(1) >= cfg.preprocess.short_test_range(2))
        error('config: short_test_range start index must be less than end index');
    end

end
