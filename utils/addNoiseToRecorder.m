function recoder_INS = addNoiseToRecorder(recoder, ins_path_simulated, ins_simulated_error)
    %ADDNOISETORECORDER 为AUV位姿及点云数据添加噪声
    % 该函数用于处理AUV多波束数据，给位姿和点云添加模拟噪声
    % 作者: Chihong(游子昂)
    % 日期: 241228
    % 版本: v1.0
    %
    % 输入参数:
    %   recoder - 原始数据记录，格式:
    %             第1列: 帧数
    %             第2-3列: AUV的xy坐标
    %             第4列: 艏向角(角度制)
    %             第5列: 标识符
    %             第6列及之后: 点云坐标(256个点的xyz)
    %   ins_path_simulated - 添加噪声后的AUV位姿，Nx3矩阵 [x,y,heading]
    %   ins_simulated_error - AUV位姿噪声，Nx3矩阵 [误差_x,误差_y,误差_heading]
    %
    % 输出参数:
    %   recoder_INS - 添加噪声后的数据记录
    %
    % 注意事项:
    %   1. 调用示例：
    %      recoder_INS = addNoiseToRecorder(recoder, ins_path_simulated, ins_simulated_error);
    %   2. 输入数据需保持帧数一致
    %

    
    % 预分配输出矩阵空间
    recoder_INS = recoder;
    
    % 获取数据维度
    [num_frames, ~] = size(recoder);
    POINTS_PER_FRAME = 256;  % 每帧点云数量
    
    % 更新AUV位姿数据
    recoder_INS(:, 2:4) = ins_path_simulated;
    
    % 处理每帧点云数据
    for frame = 1:num_frames
        % 获取当前帧的误差数据
        dx = ins_simulated_error(frame, 1);
        dy = ins_simulated_error(frame, 2);
        dheading = ins_simulated_error(frame, 3);  % 角度制
        
        % 构建变换矩阵
        % 旋转矩阵
        R = [cosd(dheading), -sind(dheading), 0;
             sind(dheading),  cosd(dheading), 0;
             0,              0,              1];
        % 平移向量
        T = [dx; dy; 0];
        
        % 提取当前帧的点云数据
        point_data = reshape(recoder(frame, 6:end), 3, POINTS_PER_FRAME);
        
        % 应用变换
        transformed_points = R * point_data + T;
        
        % 更新点云数据
        recoder_INS(frame, 6:end) = transformed_points(:)';
    end
end