clear; clc; close all;
rng(7);

%% Configuration
cfg.room = [40, 30, 12];
cfg.resolution = 1.0;
cfg.start = [2, 27, 2];
cfg.goal = [38, 3, 9];
cfg.uavRadius = 0.6;
cfg.fovDeg = 75;
cfg.ledPower = 8;
cfg.lambertOrder = 1;
cfg.pdArea = 1e-4;
cfg.pathWeights.quality = 3.5;
cfg.pathWeights.clearance = 2.0;
cfg.pathWeights.vertical = 0.35;
cfg.outputDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
    'results', 'three_dimensional_navigation');

if ~exist(cfg.outputDir, 'dir')
    mkdir(cfg.outputDir);
end

racks = [
     6 12  3  6  0  7
     6 12 10 13  0  9
     6 12 17 20  0  6
     6 12 24 27  0  8
    17 24  3  6  0  9
    17 24 10 13  0  6
    17 24 17 20  0  8
    17 24 24 27  0  7
    29 35  3  6  0  6
    29 35 10 13  0  8
    29 35 17 20  0  9
    29 35 24 27  0  6
];

ledX = linspace(3, cfg.room(1)-3, 8);
ledY = linspace(3, cfg.room(2)-3, 6);
[LX, LY] = meshgrid(ledX, ledY);
leds = [LX(:), LY(:), cfg.room(3)*ones(numel(LX), 1)];

%% Environment maps
[xv, yv, zv] = grid_axes(cfg.room, cfg.resolution);
occupancy = build_occupancy(xv, yv, zv, racks, cfg.uavRadius);
quality = build_quality_map(xv, yv, zv, leds, racks, cfg);
clearanceRisk = build_clearance_risk(xv, yv, zv, racks, cfg.uavRadius);

startNode = point_to_node(cfg.start, cfg.resolution, size(occupancy));
goalNode = point_to_node(cfg.goal, cfg.resolution, size(occupancy));
occupancy(startNode(1), startNode(2), startNode(3)) = false;
occupancy(goalNode(1), goalNode(2), goalNode(3)) = false;

%% Path planning
fprintf('Planning baseline 3D A* path...\n');
[baselineNodes, baselineInfo] = astar_3d(occupancy, startNode, goalNode, ...
    ones(size(occupancy)), zeros(size(occupancy)), 0);

fprintf('Planning communication-aware 3D A* path...\n');
[awareNodes, awareInfo] = astar_3d(occupancy, startNode, goalNode, ...
    quality, clearanceRisk, cfg.pathWeights);

assert(baselineInfo.found, 'Baseline 3D A* could not find a path.');
assert(awareInfo.found, 'Communication-aware 3D A* could not find a path.');

baselinePath = nodes_to_points(baselineNodes, cfg.resolution);
awarePath = nodes_to_points(awareNodes, cfg.resolution);
awareWaypoints = shortcut_path(awarePath, occupancy, cfg.resolution);
awareTrajectory = interpolate_path(awareWaypoints, 0.25);

%% Metrics
baselineMetrics = evaluate_path(baselinePath, quality, clearanceRisk, cfg.resolution);
awareMetrics = evaluate_path(awarePath, quality, clearanceRisk, cfg.resolution);

metricNames = ["Path length (m)"; "Mean VLC quality"; "Outage ratio"; ...
    "Mean clearance score"; "Vertical travel (m)"; "Expanded nodes"];
baselineValues = [baselineMetrics.length; baselineMetrics.meanQuality; ...
    baselineMetrics.outageRatio; baselineMetrics.meanClearance; ...
    baselineMetrics.verticalTravel; baselineInfo.expanded];
awareValues = [awareMetrics.length; awareMetrics.meanQuality; ...
    awareMetrics.outageRatio; awareMetrics.meanClearance; ...
    awareMetrics.verticalTravel; awareInfo.expanded];
metricsTable = table(metricNames, baselineValues, awareValues, ...
    'VariableNames', {'Metric', 'Baseline3DAStar', 'CommunicationAware3DAStar'});
writetable(metricsTable, fullfile(cfg.outputDir, 'metrics.csv'));
disp(metricsTable);

%% Figure 1: 3D warehouse and paths
fig1 = figure('Color', [0.035 0.045 0.07], 'Position', [80 80 1450 820]);
ax = axes(fig1);
hold(ax, 'on');
draw_warehouse(ax, cfg.room, racks, leds);

plot3(ax, baselinePath(:,1), baselinePath(:,2), baselinePath(:,3), ...
    '--', 'Color', [0.98 0.55 0.22], 'LineWidth', 2.0, ...
    'DisplayName', 'Baseline 3D A*');
plot3(ax, awareTrajectory(:,1), awareTrajectory(:,2), awareTrajectory(:,3), ...
    '-', 'Color', [0.10 0.90 0.95], 'LineWidth', 3.2, ...
    'DisplayName', 'Communication-aware 3D A*');

scatter3(ax, cfg.start(1), cfg.start(2), cfg.start(3), 130, ...
    [0.20 0.95 0.45], 'filled', 'MarkerEdgeColor', 'w', ...
    'DisplayName', 'Start');
scatter3(ax, cfg.goal(1), cfg.goal(2), cfg.goal(3), 160, ...
    [1.00 0.25 0.45], 'p', 'filled', 'MarkerEdgeColor', 'w', ...
    'DisplayName', 'Goal');

draw_uav(ax, awareTrajectory(round(size(awareTrajectory,1)*0.60),:), 0.75);
style_3d_axes(ax, cfg.room);
title(ax, 'Visible-Light-Aware 3D UAV Navigation in a Warehouse', ...
    'Color', 'w', 'FontSize', 18, 'FontWeight', 'bold');
legend(ax, 'TextColor', 'w', 'Color', [0.06 0.08 0.12], ...
    'EdgeColor', [0.35 0.4 0.5], 'Location', 'northeastoutside');
view(ax, 39, 25);
save_figure(fig1, cfg.outputDir, '01_3d_warehouse_paths');

%% Figure 2: VLC quality field and proposed path
fig2 = figure('Color', [0.035 0.045 0.07], 'Position', [100 100 1400 760]);
ax2 = axes(fig2);
hold(ax2, 'on');
qualityPlot = quality;
qualityPlot(occupancy) = NaN;
[Xplot, Yplot, Zplot] = meshgrid(xv, yv, zv);
qualityForSlice = permute(qualityPlot, [2 1 3]);
slice(ax2, Xplot, Yplot, Zplot, qualityForSlice, [8 20 32], [8 20], [3 6 9]);
shading(ax2, 'interp');
colormap(ax2, turbo(256));
cb = colorbar(ax2);
cb.Color = 'w';
cb.Label.String = 'Normalized VLC quality';
cb.Label.Color = 'w';
plot3(ax2, awareTrajectory(:,1), awareTrajectory(:,2), awareTrajectory(:,3), ...
    'w-', 'LineWidth', 3.0);
scatter3(ax2, leds(:,1), leds(:,2), leds(:,3), 35, ...
    [1.0 0.85 0.2], 'filled');
style_3d_axes(ax2, cfg.room);
title(ax2, '3D Visible-Light Quality Field and Planned Trajectory', ...
    'Color', 'w', 'FontSize', 18, 'FontWeight', 'bold');
view(ax2, 42, 26);
save_figure(fig2, cfg.outputDir, '02_vlc_quality_field');

%% Figure 3: Along-path quality comparison
fig3 = figure('Color', 'w', 'Position', [160 120 1150 620]);
tiledlayout(fig3, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
[sBase, qBase] = path_profile(baselinePath, quality, cfg.resolution);
[sAware, qAware] = path_profile(awarePath, quality, cfg.resolution);
plot(sBase, qBase, '-', 'Color', [0.95 0.45 0.15], 'LineWidth', 2.2);
hold on;
plot(sAware, qAware, '-', 'Color', [0.05 0.60 0.78], 'LineWidth', 2.5);
yline(0.20, 'k--', 'Outage threshold');
grid on;
xlabel('Travel distance (m)');
ylabel('Normalized VLC quality');
title('Communication Quality Along the Planned Paths');
legend('Baseline 3D A*', 'Communication-aware 3D A*', 'Location', 'best');

nexttile;
stairs(sBase, baselinePath(:,3), '-', 'Color', [0.95 0.45 0.15], 'LineWidth', 2.0);
hold on;
stairs(sAware, awarePath(:,3), '-', 'Color', [0.05 0.60 0.78], 'LineWidth', 2.2);
grid on;
xlabel('Travel distance (m)');
ylabel('Flight altitude (m)');
title('Altitude Profiles');
legend('Baseline 3D A*', 'Communication-aware 3D A*', 'Location', 'best');
save_figure(fig3, cfg.outputDir, '03_path_quality_profiles');

%% Figure 4: Performance comparison
fig4 = figure('Color', 'w', 'Position', [180 120 1200 620]);
tiledlayout(fig4, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
b1 = bar([baselineMetrics.length awareMetrics.length], 0.58, 'FaceColor', 'flat');
b1.CData = [0.95 0.45 0.15; 0.05 0.60 0.78];
set(gca, 'XTickLabel', {'Baseline', 'Aware'});
ylabel('Path length (m)');
title('Path Length');
grid on;

nexttile;
b2 = bar([baselineMetrics.meanQuality awareMetrics.meanQuality], 0.58, 'FaceColor', 'flat');
b2.CData = [0.95 0.45 0.15; 0.05 0.60 0.78];
set(gca, 'XTickLabel', {'Baseline', 'Aware'});
ylabel('Mean normalized quality');
title('VLC Quality');
grid on;

nexttile;
b3 = bar(100*[baselineMetrics.outageRatio awareMetrics.outageRatio], ...
    0.58, 'FaceColor', 'flat');
b3.CData = [0.95 0.45 0.15; 0.05 0.60 0.78];
set(gca, 'XTickLabel', {'Baseline', 'Aware'});
ylabel('Outage ratio (%)');
title('Communication Outage');
grid on;
save_figure(fig4, cfg.outputDir, '04_performance_comparison');

save(fullfile(cfg.outputDir, 'simulation_data.mat'), 'cfg', 'racks', 'leds', ...
    'quality', 'clearanceRisk', 'baselinePath', 'awarePath', ...
    'awareTrajectory', 'metricsTable');
fprintf('All results saved to:\n%s\n', cfg.outputDir);

%% Local functions
function [xv, yv, zv] = grid_axes(room, resolution)
    xv = 0:resolution:room(1);
    yv = 0:resolution:room(2);
    zv = 0:resolution:room(3);
end

function occupancy = build_occupancy(xv, yv, zv, boxes, margin)
    occupancy = false(numel(xv), numel(yv), numel(zv));
    for i = 1:size(boxes,1)
        b = boxes(i,:);
        xi = xv >= b(1)-margin & xv <= b(2)+margin;
        yi = yv >= b(3)-margin & yv <= b(4)+margin;
        zi = zv >= b(5) & zv <= b(6)+margin;
        occupancy(xi, yi, zi) = true;
    end
    occupancy(:,:,1) = true;
    occupancy(:,:,end) = true;
    occupancy(1,:,:) = true;
    occupancy(end,:,:) = true;
    occupancy(:,1,:) = true;
    occupancy(:,end,:) = true;
end

function quality = build_quality_map(xv, yv, zv, leds, boxes, cfg)
    [X, Y, Z] = ndgrid(xv, yv, zv);
    points = [X(:), Y(:), Z(:)];
    totalGain = zeros(size(points,1),1);
    cosFov = cosd(cfg.fovDeg);

    for i = 1:size(leds,1)
        delta = leds(i,:) - points;
        d = sqrt(sum(delta.^2,2));
        cosIncidence = delta(:,3) ./ max(d, eps);
        visible = cosIncidence >= cosFov;
        candidate = find(visible);
        visible(candidate) = ~segments_blocked_batch(points(candidate,:), leds(i,:), boxes);
        gain = cfg.ledPower * (cfg.lambertOrder+1) * cfg.pdArea .* ...
            max(cosIncidence,0).^(cfg.lambertOrder+1) ./ (2*pi*max(d,eps).^2);
        totalGain = totalGain + gain .* visible;
    end

    positive = totalGain(totalGain > 0);
    low = percentile_value(positive, 5);
    high = percentile_value(positive, 95);
    quality = (log10(totalGain + eps) - log10(low + eps)) ./ ...
        max(log10(high + eps) - log10(low + eps), eps);
    quality = min(max(quality,0),1);
    quality = reshape(quality, size(X));
end

function risk = build_clearance_risk(xv, yv, zv, boxes, margin)
    [X, Y, Z] = ndgrid(xv, yv, zv);
    minDistance = inf(size(X));
    for i = 1:size(boxes,1)
        b = boxes(i,:);
        dx = max(cat(4, b(1)-X, zeros(size(X)), X-b(2)), [], 4);
        dy = max(cat(4, b(3)-Y, zeros(size(Y)), Y-b(4)), [], 4);
        dz = max(cat(4, b(5)-Z, zeros(size(Z)), Z-b(6)), [], 4);
        minDistance = min(minDistance, sqrt(dx.^2 + dy.^2 + dz.^2));
    end
    risk = exp(-max(minDistance-margin,0)/1.8);
end

function node = point_to_node(point, resolution, mapSize)
    node = round(point/resolution) + 1;
    node = min(max(node, [1 1 1]), mapSize);
end

function points = nodes_to_points(nodes, resolution)
    points = (nodes - 1) * resolution;
end

function [path, info] = astar_3d(occupancy, startNode, goalNode, quality, risk, weights)
    mapSize = size(occupancy);
    startIdx = sub2ind(mapSize, startNode(1), startNode(2), startNode(3));
    goalIdx = sub2ind(mapSize, goalNode(1), goalNode(2), goalNode(3));

    gScore = inf(mapSize);
    fScore = inf(mapSize);
    parent = zeros(prod(mapSize),1,'uint32');
    openSet = false(mapSize);
    closedSet = false(mapSize);
    gScore(startIdx) = 0;
    fScore(startIdx) = norm(double(startNode-goalNode));
    openSet(startIdx) = true;
    expanded = 0;

    offsets = zeros(26,3);
    n = 0;
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx == 0 && dy == 0 && dz == 0
                    continue;
                end
                n = n + 1;
                offsets(n,:) = [dx dy dz];
            end
        end
    end

    found = false;
    while any(openSet(:))
        candidate = fScore;
        candidate(~openSet) = inf;
        [~, currentIdx] = min(candidate(:));
        if currentIdx == goalIdx
            found = true;
            break;
        end

        openSet(currentIdx) = false;
        closedSet(currentIdx) = true;
        expanded = expanded + 1;
        [cx, cy, cz] = ind2sub(mapSize, currentIdx);

        for k = 1:size(offsets,1)
            next = [cx cy cz] + offsets(k,:);
            if any(next < 1) || any(next > mapSize)
                continue;
            end
            nextIdx = sub2ind(mapSize, next(1), next(2), next(3));
            if occupancy(nextIdx) || closedSet(nextIdx) || ...
                    ~transition_is_free(occupancy, [cx cy cz], next)
                continue;
            end

            step = norm(offsets(k,:));
            if isstruct(weights)
                qualityPenalty = weights.quality * (1-quality(nextIdx))^2;
                riskPenalty = weights.clearance * risk(nextIdx);
                verticalPenalty = weights.vertical * abs(offsets(k,3));
                stepCost = step * (1 + qualityPenalty + riskPenalty) + verticalPenalty;
            else
                stepCost = step;
            end

            tentative = gScore(currentIdx) + stepCost;
            if tentative < gScore(nextIdx)
                parent(nextIdx) = uint32(currentIdx);
                gScore(nextIdx) = tentative;
                heuristic = norm(double(next-goalNode));
                fScore(nextIdx) = tentative + heuristic;
                openSet(nextIdx) = true;
            end
        end
    end

    path = [];
    if found
        currentIdx = goalIdx;
        while currentIdx ~= 0
            [ix, iy, iz] = ind2sub(mapSize, currentIdx);
            path = [[ix iy iz]; path]; %#ok<AGROW>
            if currentIdx == startIdx
                break;
            end
            currentIdx = double(parent(currentIdx));
        end
    end
    info = struct('found', found, 'expanded', expanded, 'cost', gScore(goalIdx));
end

function free = transition_is_free(occupancy, current, next)
    xRange = min(current(1),next(1)):max(current(1),next(1));
    yRange = min(current(2),next(2)):max(current(2),next(2));
    zRange = min(current(3),next(3)):max(current(3),next(3));
    free = ~any(occupancy(xRange,yRange,zRange), 'all');
end

function waypoints = shortcut_path(path, occupancy, resolution)
    waypoints = path(1,:);
    anchor = 1;
    while anchor < size(path,1)
        candidate = size(path,1);
        while candidate > anchor+1
            if collision_free_segment(path(anchor,:), path(candidate,:), occupancy, resolution)
                break;
            end
            candidate = candidate - 1;
        end
        waypoints = [waypoints; path(candidate,:)]; %#ok<AGROW>
        anchor = candidate;
    end
end

function free = collision_free_segment(a, b, occupancy, resolution)
    distance = norm(b-a);
    samples = max(2, ceil(distance/(0.25*resolution)));
    t = linspace(0,1,samples)';
    pts = a + t.*(b-a);
    nodes = round(pts/resolution)+1;
    mapSize = size(occupancy);
    nodes = min(max(nodes, [1 1 1]), mapSize);
    idx = sub2ind(mapSize, nodes(:,1), nodes(:,2), nodes(:,3));
    free = ~any(occupancy(idx));
end

function trajectory = interpolate_path(waypoints, spacing)
    segmentLength = sqrt(sum(diff(waypoints,1,1).^2,2));
    s = [0; cumsum(segmentLength)];
    query = (0:spacing:s(end))';
    if query(end) < s(end)
        query(end+1) = s(end);
    end
    trajectory = [interp1(s,waypoints(:,1),query,'linear'), ...
        interp1(s,waypoints(:,2),query,'linear'), ...
        interp1(s,waypoints(:,3),query,'linear')];
end

function metrics = evaluate_path(path, quality, risk, resolution)
    segment = diff(path,1,1);
    metrics.length = sum(sqrt(sum(segment.^2,2)));
    nodes = round(path/resolution)+1;
    mapSize = size(quality);
    nodes = min(max(nodes, [1 1 1]), mapSize);
    idx = sub2ind(mapSize,nodes(:,1),nodes(:,2),nodes(:,3));
    q = quality(idx);
    r = risk(idx);
    metrics.meanQuality = mean(q);
    metrics.outageRatio = mean(q < 0.20);
    metrics.meanClearance = mean(1-r);
    metrics.verticalTravel = sum(abs(segment(:,3)));
end

function [s, profile] = path_profile(path, map, resolution)
    s = [0; cumsum(sqrt(sum(diff(path,1,1).^2,2)))];
    nodes = round(path/resolution)+1;
    mapSize = size(map);
    nodes = min(max(nodes, [1 1 1]), mapSize);
    idx = sub2ind(mapSize,nodes(:,1),nodes(:,2),nodes(:,3));
    profile = map(idx);
end

function blocked = segments_blocked_batch(points, endpoint, boxes)
    blocked = false(size(points,1),1);
    direction = endpoint-points;
    for i = 1:size(boxes,1)
        b = boxes(i,:);
        boundsMin = [b(1) b(3) b(5)];
        boundsMax = [b(2) b(4) b(6)];
        tMin = zeros(size(points,1),1);
        tMax = ones(size(points,1),1);
        intersects = true(size(points,1),1);
        for axis = 1:3
            parallel = abs(direction(:,axis)) < 1e-12;
            outside = points(:,axis) < boundsMin(axis) | points(:,axis) > boundsMax(axis);
            intersects(parallel & outside) = false;

            active = ~parallel;
            t1 = zeros(size(points,1),1);
            t2 = zeros(size(points,1),1);
            t1(active) = (boundsMin(axis)-points(active,axis)) ./ direction(active,axis);
            t2(active) = (boundsMax(axis)-points(active,axis)) ./ direction(active,axis);
            tMin(active) = max(tMin(active), min(t1(active),t2(active)));
            tMax(active) = min(tMax(active), max(t1(active),t2(active)));
            intersects = intersects & tMin <= tMax;
        end
        blocked = blocked | (intersects & tMax > 1e-6 & tMin < 1-1e-6);
    end
end

function value = percentile_value(data, percentage)
    data = sort(data(:));
    if isempty(data)
        value = 0;
        return;
    end
    index = max(1,min(numel(data),round(percentage/100*numel(data))));
    value = data(index);
end

function draw_warehouse(ax, room, boxes, leds)
    floorColor = [0.08 0.11 0.16];
    patch(ax, [0 room(1) room(1) 0], [0 0 room(2) room(2)], [0 0 0 0], ...
        floorColor, 'FaceAlpha', 0.95, 'EdgeColor', [0.25 0.3 0.4]);

    for i = 1:size(boxes,1)
        h = boxes(i,6)-boxes(i,5);
        tone = 0.25 + 0.45*h/room(3);
        draw_box(ax, boxes(i,:), [0.10 tone 0.80], 0.75);
    end

    scatter3(ax, leds(:,1), leds(:,2), leds(:,3), 48, ...
        [1.0 0.83 0.15], 'filled', 'MarkerEdgeColor', [1.0 0.95 0.65], ...
        'DisplayName', 'Ceiling LEDs');
end

function draw_box(ax, box, color, alpha)
    x = [box(1) box(2)];
    y = [box(3) box(4)];
    z = [box(5) box(6)];
    vertices = [x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1); ...
                x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax, 'Vertices', vertices, 'Faces', faces, 'FaceColor', color, ...
        'FaceAlpha', alpha, 'EdgeColor', color*0.55, 'LineWidth', 0.6, ...
        'HandleVisibility', 'off');
end

function draw_uav(ax, position, scale)
    body = [0.8 0.85 0.95];
    arm = scale*[-1 1];
    plot3(ax, position(1)+arm, [position(2) position(2)], ...
        [position(3) position(3)], '-', 'Color', body, 'LineWidth', 3, ...
        'HandleVisibility', 'off');
    plot3(ax, [position(1) position(1)], position(2)+arm, ...
        [position(3) position(3)], '-', 'Color', body, 'LineWidth', 3, ...
        'HandleVisibility', 'off');
    scatter3(ax, position(1), position(2), position(3), 65, ...
        [0.1 0.9 1.0], 'filled', 'MarkerEdgeColor', 'w', ...
        'HandleVisibility', 'off');
    rotor = 0.22*scale;
    centers = [position(1)+arm(1) position(2); position(1)+arm(2) position(2); ...
               position(1) position(2)+arm(1); position(1) position(2)+arm(2)];
    theta = linspace(0,2*pi,30);
    for i = 1:4
        plot3(ax, centers(i,1)+rotor*cos(theta), centers(i,2)+rotor*sin(theta), ...
            position(3)*ones(size(theta)), '-', 'Color', [0.35 0.95 1], ...
            'LineWidth', 1.3, 'HandleVisibility', 'off');
    end
end

function style_3d_axes(ax, room)
    axis(ax, [0 room(1) 0 room(2) 0 room(3)]);
    axis(ax, 'equal');
    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'X (m)', 'Color', 'w');
    ylabel(ax, 'Y (m)', 'Color', 'w');
    zlabel(ax, 'Z (m)', 'Color', 'w');
    ax.Color = [0.035 0.045 0.07];
    ax.XColor = [0.75 0.80 0.90];
    ax.YColor = [0.75 0.80 0.90];
    ax.ZColor = [0.75 0.80 0.90];
    ax.GridColor = [0.45 0.55 0.70];
    ax.GridAlpha = 0.20;
    camproj(ax, 'perspective');
end

function save_figure(fig, outputDir, name)
    drawnow;
    exportgraphics(fig, fullfile(outputDir, [name '.png']), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, [name '.fig']));
end
