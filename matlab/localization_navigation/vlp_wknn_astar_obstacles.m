clear; clc; close all;

%% 参数设置
fprintf('========================================\n');
fprintf('可见光定位与航迹规划仿真系统（考虑不可透光障碍物）\n');
fprintf('========================================\n\n');

room_size = [30, 20, 8];        % [长, 宽, 高] 米
grid_resolution = 0.5;          % 指纹库与A*地图的栅格分辨率 (m)
grid_cols = round(room_size(1) / grid_resolution);
grid_rows = round(room_size(2) / grid_resolution);
grid_spacing = grid_resolution;

led_x = linspace(3, room_size(1)-3, 5);
led_y = linspace(2.5, room_size(2)-2.5, 4);
[LED_X, LED_Y] = meshgrid(led_x, led_y);
LED_pos = [LED_X(:), LED_Y(:), room_size(3)*ones(numel(LED_X),1)];
num_LED = size(LED_pos, 1);
LED_freq = 800 + (0:num_LED-1) * 150;  % 每个LED使用独立调制频率

P_tx     = 3;           % LED发射功率 (W)
m_lambert = 1;          % 朗伯辐射模型阶数 (对应半功率角60度)

A_pd   = 1e-4;          % PD物理面积 (1cm² = 1e-4 m²)
FOV    = 70;            % 视场角 (度)
FOV_rad = deg2rad(FOV); % 转换为弧度
R_pd   = 0.5;           % PD响应度 (A/W)
Ts     = 1;             % 光学滤波器增益
n_refract = 1.5;        % 光学集中器折射率
g_con  = n_refract^2 / sin(FOV_rad)^2;  % 光学集中器增益 ≈ 2.55

rho_wall = 0.8;         % 墙面反射率
N_wall   = 10;          % 墙面离散化
enable_NLOS = false;    % 大型仓库默认关闭NLOS以保证直接运行速度
Fs       = 120e3;       % 采样频率 (120 kHz)
B        = Fs / 2;      % 系统带宽 (60 kHz)
N_DFT    = 1024;        % DFT点数
q        = 1.602e-19;   % 基本电荷 (C)
k_B      = 1.38e-23;    % 玻尔兹曼常数 (J/K)
T_abs    = 295;         % 绝对温度 (K)
R_L      = 10e3;        % 负载电阻 (10 kΩ)
I_bg     = 260e-6;      % 背景光电流 (260 μA)

K_values = [3, 5, 7];   % 测试不同K值
num_test = 100;         % 测试点数量

%% 仓库与航迹规划场景
fprintf('\n--- 默认仓库航迹规划设置 ---\n');

obstacles = [5, 12, 3, 4.5;
             5, 12, 7, 8.5;
             5, 12, 11, 12.5;
             5, 12, 15, 16.5;
             17, 25, 3, 4.5;
             17, 25, 7, 8.5;
             17, 25, 11, 12.5;
             17, 25, 15, 16.5];

Demo_start_true = [2, 18, 0];  % 可复现的演示起点
Goal_pos_meters = [28, 2];     % 默认目标点

fprintf('\n参数设置完成!\n');
fprintf('仓库尺寸: %.0fm × %.0fm × %.0fm\n', room_size);
fprintf('LED数量: %d, 地图栅格: %d × %d\n', num_LED, grid_cols, grid_rows);
fprintf('开始构建指纹库...\n\n');

%% 离线阶段：构建指纹库

x_grid = ((1:grid_cols) - 0.5) * grid_spacing;
y_grid = ((1:grid_rows) - 0.5) * grid_spacing;
[X_RP, Y_RP] = meshgrid(x_grid, y_grid);
RP_coords = [X_RP(:), Y_RP(:), zeros(numel(X_RP), 1)];
num_RP = size(RP_coords, 1);

valid_RP = true(num_RP, 1);
for i_obs = 1:size(obstacles, 1)
    obs  = obstacles(i_obs, :);    % [xmin xmax ymin ymax]
    xmin = obs(1); xmax = obs(2);
    ymin = obs(3); ymax = obs(4);

    inside = RP_coords(:,1) >= xmin & RP_coords(:,1) <= xmax & ...
             RP_coords(:,2) >= ymin & RP_coords(:,2) <= ymax;
    valid_RP = valid_RP & ~inside;
end
RP_coords = RP_coords(valid_RP, :);    % 只保留有效参考点
num_RP    = size(RP_coords, 1);

fingerprint_DB = zeros(num_RP, 2 + num_LED);  % [x, y, RSS1 ... RSSn]
fingerprint_DB(:, 1:2) = RP_coords(:, 1:2);

wall_elements = generate_wall_elements(room_size, N_wall);

fprintf('构建指纹库进度:\n');
h = waitbar(0, '计算参考点指纹...');

for i = 1:num_RP
    RP_pos = RP_coords(i, :);
    RSS_vector = zeros(1, num_LED);
    I_sig_total = 0;

    for k = 1:num_LED
        LED_k = LED_pos(k, :);

        H_LOS = calculate_LOS_gain(LED_k, RP_pos, m_lambert, A_pd, ...
                                   Ts, g_con, FOV_rad, obstacles);

        H_NLOS = 0;
        if enable_NLOS
            H_NLOS = calculate_NLOS_gain(LED_k, RP_pos, wall_elements, ...
                                         rho_wall, m_lambert, A_pd, Ts, g_con, FOV_rad);
        end

        H_total = H_LOS + H_NLOS;

        P_rx = P_tx * H_total;
        I_sig = R_pd * P_rx;
        I_sig_total = I_sig_total + I_sig;

        RSS_vector(k) = I_sig;
    end

    sigma_shot2    = 2 * q * (I_sig_total + I_bg) * B;
    sigma_thermal2 = (4 * k_B * T_abs / R_L) * B;
    sigma_total2   = sigma_shot2 + sigma_thermal2;
    sigma_bin      = sqrt(sigma_total2 / (N_DFT / 2));

    RSS_noisy = RSS_vector + sigma_bin * randn(1, num_LED);
    fingerprint_DB(i, 3:end) = RSS_noisy;

    if mod(i, 10) == 0 || i == num_RP
        waitbar(i/num_RP, h, ...
            sprintf('已完成: %d/%d (%.1f%%)', i, num_RP, 100*i/num_RP));
    end
end

close(h);
fprintf('指纹库构建完成! (有效参考点数量: %d)\n\n', num_RP);

%% 在线阶段：WKNN定位测试

fprintf('开始在线定位测试...\n');
fprintf('测试点数量: %d\n', num_test);
fprintf('K值范围: %s\n\n', mat2str(K_values));

results = struct();
for k_idx = 1:length(K_values)
    K = K_values(k_idx);
    results(k_idx).K       = K;
    results(k_idx).errors  = zeros(num_test, 1);
    results(k_idx).true_pos = zeros(num_test, 2);
    results(k_idx).est_pos  = zeros(num_test, 2);
end

h = waitbar(0, '定位测试中...');

for t = 1:num_test
    margin = 1.0;
    while true
        if t == num_test
            TP_candidate = Demo_start_true;
        else
            TP_candidate = [margin + (room_size(1)-2*margin)*rand(), ...
                            margin + (room_size(2)-2*margin)*rand(), 0];
        end

        is_in_obstacle = false;
        for obs_idx = 1:size(obstacles, 1)
            obs = obstacles(obs_idx, :); % [xmin xmax ymin ymax]
            if (TP_candidate(1) >= obs(1) && TP_candidate(1) <= obs(2) && ...
                TP_candidate(2) >= obs(3) && TP_candidate(2) <= obs(4))
                is_in_obstacle = true;
                break;
            end
        end

        if ~is_in_obstacle
            TP_true = TP_candidate;
            break;
        end
    end

    RSS_measured = zeros(1, num_LED);
    I_sig_total = 0;

    for k = 1:num_LED
        LED_k = LED_pos(k, :);

        H_LOS = calculate_LOS_gain(LED_k, TP_true, m_lambert, A_pd, ...
                                   Ts, g_con, FOV_rad, obstacles);

        H_NLOS = 0;
        if enable_NLOS
            H_NLOS = calculate_NLOS_gain(LED_k, TP_true, wall_elements, rho_wall, ...
                                         m_lambert, A_pd, Ts, g_con, FOV_rad);
        end

        H_total = H_LOS + H_NLOS;
        P_rx    = P_tx * H_total;
        I_sig   = R_pd * P_rx;
        I_sig_total = I_sig_total + I_sig;
        RSS_measured(k) = I_sig;
    end

    sigma_shot2    = 2 * q * (I_sig_total + I_bg) * B;
    sigma_thermal2 = (4 * k_B * T_abs / R_L) * B;
    sigma_total2   = sigma_shot2 + sigma_thermal2;
    sigma_bin      = sqrt(sigma_total2 / (N_DFT / 2));
    RSS_measured   = RSS_measured + sigma_bin * randn(1, num_LED);

    for k_idx = 1:length(K_values)
        K = K_values(k_idx);
        TP_est = WKNN_positioning(RSS_measured, fingerprint_DB, K);

        error_val = norm(TP_true(1:2) - TP_est);
        results(k_idx).errors(t)   = error_val;
        results(k_idx).true_pos(t, :) = TP_true(1:2);
        results(k_idx).est_pos(t, :)  = TP_est;
    end

    if mod(t, 10) == 0 || t == num_test
        waitbar(t/num_test, h, sprintf('测试进度: %d/%d', t, num_test));
    end
end

close(h);
fprintf('定位测试完成!\n\n');

%% A* 航迹规划

fprintf('开始A*航迹规划...\n');

AStar_map = zeros(grid_rows, grid_cols);

for i = 1:size(obstacles, 1)
    obs = obstacles(i, :);
    x_min_idx = max(1, ceil(obs(1) / grid_spacing));
    x_max_idx = min(grid_cols, ceil(obs(2) / grid_spacing));
    y_min_idx = max(1, ceil(obs(3) / grid_spacing));
    y_max_idx = min(grid_rows, ceil(obs(4) / grid_spacing));

    AStar_map(y_min_idx:y_max_idx, x_min_idx:x_max_idx) = 1;
end

K_demo_idx = 2;  % 对应 K=5
Start_pos_meters = results(K_demo_idx).est_pos(end, :); % [x, y]
Goal_pos_meters_plot = Goal_pos_meters;                 % [x, y]

Start_node = [min(grid_rows, max(1, ceil(Start_pos_meters(2) / grid_spacing))), ...
              min(grid_cols, max(1, ceil(Start_pos_meters(1) / grid_spacing)))];

Goal_node  = [min(grid_rows, max(1, ceil(Goal_pos_meters_plot(2) / grid_spacing))), ...
              min(grid_cols, max(1, ceil(Goal_pos_meters_plot(1) / grid_spacing)))];

Start_node = nearest_free_node(AStar_map, Start_node);
Goal_node = nearest_free_node(AStar_map, Goal_node);

if AStar_map(Start_node(1), Start_node(2)) == 1
    fprintf('警告: 起点在障碍物内! 规划可能失败。\n');
end
if AStar_map(Goal_node(1), Goal_node(2)) == 1
    fprintf('警告: 终点在障碍物内! 规划可能失败。\n');
end

[path_indices, path_found] = AStar_pathfinding(AStar_map, Start_node, Goal_node);

path_meters = [];
if path_found
    fprintf('A* 路径已找到! (共 %d 个步骤)\n', size(path_indices, 1));
    path_rows = path_indices(:, 1);
    path_cols = path_indices(:, 2);
    path_meters = [(path_cols - 0.5) * grid_spacing, ...
                   (path_rows - 0.5) * grid_spacing];
else
    fprintf('A* 未找到路径!\n');
end

fprintf('航迹规划完成!\n\n');

%% 结果可视化与保存

fprintf('生成可视化结果...\n');

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, '..', '..', 'results', 'warehouse_navigation');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

fig1 = figure('Name', 'VLP定位与A*航迹规划', 'Position', [100, 100, 1000, 700]);
hold on; grid on; axis equal;

plot(fingerprint_DB(:,1), fingerprint_DB(:,2), 'k.', 'MarkerSize', 8, ...
     'DisplayName', '参考点 (有效)');

plot(LED_pos(:,1), LED_pos(:,2), 'rs', 'MarkerSize', 15, ...
     'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'LED发射器');

for i = 1:num_LED
    text(LED_pos(i,1)+0.15, LED_pos(i,2), sprintf('LED%d\n%.0fHz', i, LED_freq(i)), ...
         'FontSize', 10, 'FontWeight', 'bold');
end

for i = 1:size(obstacles, 1)
    obs = obstacles(i, :);
    x = obs(1);
    y = obs(3);
    w = obs(2) - obs(1);
    h = obs(4) - obs(3);
    rectangle('Position', [x, y, w, h], 'FaceColor', [0.5 0.5 0.5], ...
              'EdgeColor', 'k', 'LineWidth', 1);
end
plot(NaN, NaN, 's', 'MarkerFaceColor', [0.5 0.5 0.5], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1, 'MarkerSize', 10, ...
    'DisplayName', '障碍物');

K_demo       = K_values(K_demo_idx);
true_pos_demo = results(K_demo_idx).true_pos(end, :);
est_pos_demo  = results(K_demo_idx).est_pos(end, :);

plot(true_pos_demo(1), true_pos_demo(2), 'rx', 'MarkerSize', 15, ...
     'LineWidth', 3, 'DisplayName', sprintf('真实位置 (测试点%d)', num_test));
plot(est_pos_demo(1), est_pos_demo(2), 'bo', 'MarkerSize', 12, ...
     'LineWidth', 2, 'DisplayName', sprintf('VLP估计位置 (K=%d)', K_demo));
plot([true_pos_demo(1), est_pos_demo(1)], ...
     [true_pos_demo(2), est_pos_demo(2)], ...
     'b--', 'LineWidth', 1.5, 'DisplayName', 'VLP定位误差');

plot(Goal_pos_meters_plot(1), Goal_pos_meters_plot(2), 'gh', 'MarkerSize', 16, ...
     'LineWidth', 2, 'MarkerFaceColor', 'g', 'DisplayName', 'A* 目标点');

if path_found && ~isempty(path_meters)
    plot(path_meters(:, 1), path_meters(:, 2), 'm-o', 'LineWidth', 2, ...
         'MarkerSize', 6, 'MarkerFaceColor', 'm', 'DisplayName', 'A* 规划路径');
end

xlabel('X 坐标 (m)', 'FontSize', 12);
ylabel('Y 坐标 (m)', 'FontSize', 12);
title('VLP定位与A*航迹规划（不可透光障碍物）', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'bestoutside');
xlim([0, room_size(1)]); ylim([0, room_size(2)]);
set(gca, 'FontSize', 11);
hold off;
save_result_figure(fig1, results_dir, '01_navigation_path');

fig2 = figure('Name', 'RSS指纹分布', 'Position', [150, 50, 1400, 420]);
num_heatmaps = min(4, num_LED);
for led_idx = 1:num_heatmaps
    subplot(1, num_heatmaps, led_idx);
    [Xg, Yg] = meshgrid(x_grid, y_grid);
    RSS_vals = fingerprint_DB(:, 2+led_idx);
    Zg = griddata(fingerprint_DB(:,1), fingerprint_DB(:,2), RSS_vals, Xg, Yg, 'natural');
    imagesc(x_grid, y_grid, Zg);
    colorbar; axis equal tight;
    set(gca, 'YDir', 'normal');
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('LED%d (%.0fHz) RSS分布', led_idx, LED_freq(led_idx)));
    hold on;
    plot(LED_pos(led_idx,1), LED_pos(led_idx,2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
end
save_result_figure(fig2, results_dir, '02_rss_fingerprint_maps');

fig3 = figure('Name', 'VLP定位误差CDF', 'Position', [200, 100, 800, 600]);
hold on; grid on;
colors = ['r', 'b', 'g'];
for k_idx = 1:length(K_values)
    errors_sorted = sort(results(k_idx).errors);
    cdf = (1:num_test) / num_test;
    plot(errors_sorted, cdf, [colors(k_idx), '-'], 'LineWidth', 2.5, ...
         'DisplayName', sprintf('K=%d', K_values(k_idx)));

    mean_err = mean(results(k_idx).errors);
    p90_index = max(1, ceil(0.9 * num_test));
    p90_err  = errors_sorted(p90_index);
    fprintf('K=%d (VLP): 平均误差=%.3fm, 90%%误差=%.3fm, 最大误差=%.3fm\n', ...
            K_values(k_idx), mean_err, p90_err, max(results(k_idx).errors));
end
xlabel('定位误差 (m)', 'FontSize', 13);
ylabel('累积分布函数 (CDF)', 'FontSize', 13);
title('不同K值的VLP定位误差CDF对比', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 11);
xlim([0, max(results(2).errors)*1.1]);
set(gca, 'FontSize', 11);
save_result_figure(fig3, results_dir, '03_positioning_error_cdf');

fig4 = figure('Name', 'VLP误差统计对比', 'Position', [250, 150, 700, 500]);
mean_errors = zeros(size(K_values));
p90_errors = zeros(size(K_values));
for k_idx = 1:length(K_values)
    sorted_errors = sort(results(k_idx).errors);
    mean_errors(k_idx) = mean(sorted_errors);
    p90_errors(k_idx) = sorted_errors(max(1, ceil(0.9 * num_test)));
end
bar(K_values, [mean_errors; p90_errors]');
xlabel('K值', 'FontSize', 13);
ylabel('定位误差 (m)', 'FontSize', 13);
title('不同K值的VLP误差统计', 'FontSize', 14, 'FontWeight', 'bold');
legend('平均误差', '90%误差', 'Location', 'northwest');
grid on;
set(gca, 'FontSize', 11);
save_result_figure(fig4, results_dir, '04_positioning_error_summary');

fprintf('结果图片已保存至: %s\n', results_dir);

%% 局部函数

function wall_elems = generate_wall_elements(room, N)
    L = room(1); W = room(2); H = room(3);
    dL = L / N; dW = W / N; dH = H / N;
    dA = dL * dH;  % 或 dW * dH
    wall_elems = [];

    [Y1, Z1] = meshgrid(linspace(dW/2, W-dW/2, N), linspace(dH/2, H-dH/2, N));
    wall1 = [zeros(N^2,1), Y1(:), Z1(:), ...
             ones(N^2,1), zeros(N^2,2), ...
             dA*ones(N^2,1), ones(N^2,1)];
    wall2 = [L*ones(N^2,1), Y1(:), Z1(:), ...
             -ones(N^2,1), zeros(N^2,2), ...
             dA*ones(N^2,1), 2*ones(N^2,1)];
    [X3, Z3] = meshgrid(linspace(dL/2, L-dL/2, N), linspace(dH/2, H-dH/2, N));
    wall3 = [X3(:), zeros(N^2,1), Z3(:), ...
             zeros(N^2,1), ones(N^2,1), zeros(N^2,1), ...
             dA*ones(N^2,1), 3*ones(N^2,1)];
    wall4 = [X3(:), W*ones(N^2,1), Z3(:), ...
             zeros(N^2,1), -ones(N^2,1), zeros(N^2,1), ...
             dA*ones(N^2,1), 4*ones(N^2,1)];

    wall_elems = [wall1; wall2; wall3; wall4];
    wall_elems(:, 9) = 1;  % 有效标志
end

function H = calculate_LOS_gain(LED_pos, PD_pos, m, A_pd, Ts, g, FOV, obstacles)
    if nargin >= 8 && ~isempty(obstacles)
        if is_blocked_by_obstacles_2D(LED_pos, PD_pos, obstacles)
            H = 0;
            return;
        end
    end

    vec = PD_pos - LED_pos;
    d = norm(vec);
    if d < 1e-6
        H = 0;
        return;
    end

    cos_phi = abs(vec(3)) / d;
    cos_psi = abs(vec(3)) / d;
    psi = acos(cos_psi);

    if psi > FOV
        H = 0;
        return;
    end

    g_psi = (psi <= FOV) * g;
    H = ((m+1) * A_pd / (2*pi*d^2)) * (cos_phi^m) * Ts * g_psi * cos_psi;
end

function H_NLOS = calculate_NLOS_gain(LED_pos, PD_pos, wall_elems, rho, m, A_pd, Ts, g, FOV)
    H_NLOS = 0;
    num_elems = size(wall_elems, 1);

    for i = 1:num_elems
        elem_pos    = wall_elems(i, 1:3);
        elem_normal = wall_elems(i, 4:6);
        dA          = wall_elems(i, 7);

        vec1 = elem_pos - LED_pos;
        d1   = norm(vec1);
        if d1 < 1e-6
            continue;
        end
        cos_phi1 = abs(vec1(3)) / d1;
        cos_psi1 = -dot(vec1, elem_normal) / d1;
        if cos_psi1 <= 0
            continue;
        end

        vec2 = PD_pos - elem_pos;
        d2   = norm(vec2);
        if d2 < 1e-6
            continue;
        end
        cos_phi2 = dot(vec2, elem_normal) / d2;
        cos_psi2 = abs(vec2(3)) / d2;
        psi2     = acos(cos_psi2);
        if cos_phi2 <= 0 || psi2 > FOV
            continue;
        end

        g_psi2 = (psi2 <= FOV) * g;

        dH = rho * dA * ((m+1)/(2*pi*d1^2)) * (cos_phi1^m) * cos_psi1 * ...
             (A_pd / (pi*d2^2)) * cos_phi2 * Ts * g_psi2 * cos_psi2;
        H_NLOS = H_NLOS + dH;
    end
end

function pos_est = WKNN_positioning(RSS_measured, DB, K)
    RSS_db = DB(:, 3:end);
    distances = sqrt(sum((RSS_db - repmat(RSS_measured, size(RSS_db,1), 1)).^2, 2));
    [~, idx_sorted] = sort(distances);
    idx_KNN = idx_sorted(1:K);

    epsilon = 1e-9;
    weights = 1 ./ (distances(idx_KNN) + epsilon);
    weights = weights / sum(weights);

    pos_est = sum(weights .* DB(idx_KNN, 1:2), 1);
end

function free_node = nearest_free_node(map, node)
    if map(node(1), node(2)) == 0
        free_node = node;
        return;
    end

    [free_rows, free_cols] = find(map == 0);
    [~, idx] = min((free_rows - node(1)).^2 + (free_cols - node(2)).^2);
    free_node = [free_rows(idx), free_cols(idx)];
end

function [path_indices, path_found] = AStar_pathfinding(map, start_node, goal_node)

    [rows, cols] = size(map);
    map_size = [rows, cols];

    start_ind = sub2ind(map_size, start_node(1), start_node(2));
    goal_ind  = sub2ind(map_size, goal_node(1), goal_node(2));

    gScore = Inf(rows, cols);
    gScore(start_ind) = 0;

    fScore = Inf(rows, cols);

    [r, c] = ind2sub(map_size, (1:rows*cols)');
    goal_r = goal_node(1);
    goal_c = goal_node(2);
    h = sqrt((r - goal_r).^2 + (c - goal_c).^2);
    h_matrix = reshape(h, rows, cols);
    fScore(start_ind) = h_matrix(start_ind);

    openSet   = false(rows, cols);
    openSet(start_ind) = true;
    closedSet = false(rows, cols);

    cameFrom  = zeros(rows*cols, 1);  % 父节点线性索引
    path_found = false;

    while any(openSet(:))
        fScore_open = fScore;
        fScore_open(~openSet) = Inf;
        [~, current_ind] = min(fScore_open(:));
        [current_r, current_c] = ind2sub(map_size, current_ind);

        if current_ind == goal_ind
            path_found = true;
            break;
        end

        openSet(current_ind)   = false;
        closedSet(current_ind) = true;

        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0
                    continue;
                end

                neighbor_r = current_r + dr;
                neighbor_c = current_c + dc;

                if neighbor_r < 1 || neighbor_r > rows || ...
                   neighbor_c < 1 || neighbor_c > cols
                    continue;
                end

                neighbor_ind = sub2ind(map_size, neighbor_r, neighbor_c);

                if map(neighbor_ind) == 1
                    continue;
                end

                if closedSet(neighbor_ind)
                    continue;
                end

                move_cost = sqrt(dr^2 + dc^2); % 1 或 sqrt(2)
                tentative_gScore = gScore(current_ind) + move_cost;

                if tentative_gScore < gScore(neighbor_ind)
                    cameFrom(neighbor_ind) = current_ind;
                    gScore(neighbor_ind)   = tentative_gScore;
                    fScore(neighbor_ind)   = tentative_gScore + h_matrix(neighbor_ind);

                    if ~openSet(neighbor_ind)
                        openSet(neighbor_ind) = true;
                    end
                end
            end
        end
    end

    if path_found
        path_linear = reconstructPath_AStar(cameFrom, current_ind);
        [path_rows, path_cols] = ind2sub(map_size, path_linear);
        path_indices = [path_rows, path_cols];
    else
        path_indices = [];
    end
end

function path_indices = reconstructPath_AStar(cameFrom, current_ind)
    total_path = current_ind;
    while cameFrom(current_ind) ~= 0
        current_ind = cameFrom(current_ind);
        total_path  = [current_ind; total_path]; %#ok<AGROW>
    end
    path_indices = total_path;
end

function blocked = is_blocked_by_obstacles_2D(LED_pos, PD_pos, obstacles)
    p1 = LED_pos(1:2);  % [xL, yL]
    p2 = PD_pos(1:2);   % [xP, yP]
    blocked = false;

    if isempty(obstacles)
        return;
    end

    for i = 1:size(obstacles, 1)
        obs  = obstacles(i, :);  % [xmin xmax ymin ymax]
        xmin = obs(1); xmax = obs(2);
        ymin = obs(3); ymax = obs(4);

        if (p1(1) >= xmin && p1(1) <= xmax && p1(2) >= ymin && p1(2) <= ymax) || ...
           (p2(1) >= xmin && p2(1) <= xmax && p2(2) >= ymin && p2(2) <= ymax)
            blocked = true;
            return;
        end

        rect = [ xmin  ymin;
                 xmax  ymin;
                 xmax  ymax;
                 xmin  ymax ];
        for e = 1:4
            a = rect(e, :);
            b = rect(mod(e,4)+1, :);
            if segments_intersect_2D(p1, p2, a, b)
                blocked = true;
                return;
            end
        end
    end
end

function flag = segments_intersect_2D(p1, p2, p3, p4)
    d1 = direction_2D(p3, p4, p1);
    d2 = direction_2D(p3, p4, p2);
    d3 = direction_2D(p1, p2, p3);
    d4 = direction_2D(p1, p2, p4);

    flag = (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ...
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
end

function d = direction_2D(p1, p2, p3)
    d = (p3(1) - p1(1)) * (p2(2) - p1(2)) - ...
        (p3(2) - p1(2)) * (p2(1) - p1(1));
end

function save_result_figure(fig, output_dir, filename)
    drawnow;
    exportgraphics(fig, fullfile(output_dir, [filename '.png']), 'Resolution', 200);
    savefig(fig, fullfile(output_dir, [filename '.fig']));
end
