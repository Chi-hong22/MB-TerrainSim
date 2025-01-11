%% multibeam - 多波束声呐仿真采集函数
%
% 功能描述：
%   模拟多波束声呐对海底地形的扫描采集过程，基于射线追踪算法实现水下地形点云的生成
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
%   v1.2 (240409) - 性能优化
%       + 添加并行计算支持
%       + 优化内存使用
%   v1.1 (240408) - 功能完善
%       + 添加射线追踪算法
%       + 实现三角面片相交检测
%   v1.0 (240407) - 首次发布
%       + 实现基础多波束仿真功能
%
% 输入参数：
%   X        - [MxN double] 地形网格X坐标矩阵
%   Y        - [MxN double] 地形网格Y坐标矩阵
%   Z        - [MxN double] 地形深度矩阵
%   poseList - [Kx3 double] AUV位姿列表 [x y heading]
%   depth    - [double] AUV深度（m）
%   range    - [double] 声呐最大测量距离（m）
%   angle    - [double] 声呐扫描角度（度）
%   beamnum  - [int] 波束数量
%
% 输出参数：
%   recoder  - [KxM double] 采集记录矩阵
%              第1-5列：[帧号 x y heading depth]
%              第6列之后：波束采集点坐标 [x1 y1 z1 ... xn yn zn]
%
% 注意事项：
%   1. 要求输入地形网格为规则网格
%   2. 波束数量会影响计算性能
%   3. 建议使用并行计算以提高性能
%
% 调用示例：
%   % 基础调用
%   recoder = multibeam(X, Y, Z, poses, -5, 100, 60, 256);
%
% 依赖函数：
%   - TriangleRayIntersection
%   - getPart (内部函数)

function recoder = multibeam(X, Y, Z, poseList, depth, range, angle, beamnum)

    posenum = size(poseList, 1);
    angle_res = 2*angle/(beamnum - 1);

    recoder = zeros(posenum, length(-angle:angle_res:angle)*3 + 5);
    step = 1;
    [X, Y, Z] = deal(X(1:step:end, 1:step:end), Y(1:step:end, 1:step:end), Z(1:step:end, 1:step:end));


    maxdistance = abs(min(min(Z)) - depth);
    [rawsize, colsize] = size(X);

    if X(1, 1) == X(1, 2)
        map_res = abs(X(1, 1) - X(2, 1));
        xflag = 1;
    else
        map_res = abs(X(1, 1) - X(1, 2));
        xflag = 0;
    end

    rawcolRange = ceil(maxdistance*tand(angle)/map_res) + 1;

    parfor i = 1:posenum
        heading_i = poseList(i, 3);
        orig = [poseList(i, 1:2), depth];

        tempt = zeros(1, length(-angle:angle_res:angle)*3 + 5);
        tempt(1, 1:5) = [i, poseList(i, 1:3), depth];

        count = 0;

        if xflag == 0
            col_ = round((orig(1) - X(1, 1))/(X(1, 2) - X(1, 1)));
            raw_ = round((orig(2) - Y(1, 1))/(Y(2, 1) - Y(1, 1)));
        else
            raw_ = round((orig(1) - X(1, 1))/map_res);
            col_ = round((orig(2) - Y(1, 1))/map_res);
        end

        raw_min = max(raw_-rawcolRange, 1);
        raw_max = min(raw_+rawcolRange, rawsize);

        col_min = max(col_-rawcolRange, 1);
        col_max = min(col_+rawcolRange, colsize);

        Xi = X(raw_min:raw_max, col_min:col_max);
        Yi = Y(raw_min:raw_max, col_min:col_max);
        Zi = Z(raw_min:raw_max, col_min:col_max);

        choosepart = getPart(Xi, Yi, orig(1:2), heading_i, map_res, 3);
        vertices = [Xi(choosepart) Yi(choosepart) Zi(choosepart)];
        faces = delaunay(Xi(choosepart), Yi(choosepart));

        vert1 = vertices(faces(:,1),:);
        vert2 = vertices(faces(:,2),:);
        vert3 = vertices(faces(:,3),:);

    %     surf(Xi, Yi, Zi );
    %     colormap("jet")
    %     shading interp
    % 
    %     hold on;
    %     plot3(Xi(choosepart), Yi(choosepart), Zi(choosepart), 'ko')
        
        for theta = -angle:angle_res:angle
            count = count + 1;

            yb = -range*sind(theta);
            zb = -range*cosd(theta);

            dir = [-yb*sind(heading_i), yb*cosd(heading_i), zb];
            [~, ~, ~, ~, point] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'fullReturn', false, 'lineType', 'ray');
            tempt(1, 5 + count*3-2: 5 + count*3) = point;

    %         hold on; plot3(point(1), point(2), point(3), 'r*');
    %         hold on; plot3([orig(1), orig(1)+dir(1)], [orig(2), orig(2)+dir(2)], [orig(3), orig(3)+dir(3)], 'r');
    %         hold on; plot3(orig(1), orig(2), orig(3), 'ko');
        end

    %     hold off;
        recoder(i, :) = tempt;
        i

    end
end

function choosePart = getPart(X, Y, origin, psi, res, num)
    x = origin(1);
    y = origin(2);
    disThreshold = num*res*1.6;

    rho = x*cosd(psi) + y*sind(psi);
    distance = abs(X*cosd(psi) + Y*sind(psi) - rho);
    choosePart = distance < disThreshold;
end