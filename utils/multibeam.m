function recoder = multibeam(X, Y, Z, poseList, depth, range, angle, beamnum)
%MULTIBEAM 此处显示有关此函数的摘要
%   此处显示详细说明


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