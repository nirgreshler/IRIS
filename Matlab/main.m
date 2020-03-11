close all
clc
clear

rng(1);

% env_name = 'planar_1000';
num_rooms = 4;
env_name = ['syn_' num2str(num_rooms) 'rooms'];

K_MEANS_START = 2;
K_MEANS_END = 10;
k_vec = K_MEANS_START:K_MEANS_END;
CVG_TH = 0.8;
addpath(genpath(pwd))

SPECTRAL = true;
clusteringMethod = 'spectral'; % 'kmeans' / 'spectral' / 'inspection'

% rng(K_MEANS_START);

% PLOTTING
PLOT_ENVIRONMENMT = false;
PLOT_CLUSTERING = false;
PLOT_HISTOGRAMS = false;

% SOLVING
RUN_SEARCH = true;
initial_p = 0.8;
initial_eps = 0.8;
tightening_rate = 0;
method = 0;

dist = 'sqeuclidean';
base_name = fullfile(pwd, 'Graphs');
username = getenv('USERNAME');
username = strrep(username, '.', '');
if strcmp(username, 'Gal')
    username = 'galgreshler';
end
if strcmp(username, 'Nir')
    username = 'nirgreshler';
end
wsl_path = ['/home/' username '/Project/IRIS'];
search_path = [wsl_path, '/debug/app/search_graph'];
base_name_in_wsl = ['/mnt/' lower(strrep(strrep(base_name,':',''),'\','/'))];

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));

params.maxClusters = 10;
params.minClusters = 3;
params.showText = false;

% Perform clustering
% M = Edges2M(edges(:,[1 2 7]));
% switch clusteringMethod
%     case 'kmeans'
%         clusters = KMeansClustering(params, conf(:,2:3), num_rooms);
%     case 'spectral'
%         clusters = SpectralClustering(params, conf(:,2:3), M, num_rooms);
%     case 'inspection'
%         pointsInSight = GetPointsInSight(params, conf(:,2:3), inspectionPoints, obstacles);
%         clusters = InspectionClustering(params, conf(:,2:3), pointsInSight);
% end

% build the bridge graph
% [bridge_conf, bridge_vertex, bridge_edges] = get_bridge_graph(conf, vertex, edges, clusters, obstacles, params);

% plotGraph(bridge_conf, bridge_vertex, bridge_edges);
% title('Bridge graph BEFORE inner edges');

step_in_bridge_search = 10;
% save bridge graphs after each RRT iteration(s)
folder = fullfile(base_name, [env_name '_bridges']);
if ~exist(folder, 'dir')
    mkdir(folder);
else
    rmdir(folder, 's');
    pause(2);
    mkdir(folder);
end
params.plotEdges = true;
for i = step_in_bridge_search:step_in_bridge_search:size(conf, 1)
    rrt_conf = conf(1:i, :);
    rrt_vertex = vertex(1:i, :);
    rrt_edges = get_edges_per_vertex(edges, rrt_vertex(:, 1));
    
    M = Edges2M(rrt_edges(:, [1:2,7]), size(rrt_vertex, 1));

    switch clusteringMethod
        case 'kmeans'
            clusters = KMeansClustering(params, rrt_conf(:,2:3), num_rooms);
        case 'spectral'
            clusters = SpectralClustering(params, rrt_conf(:,2:3), M);
        case 'inspection'
            pointsInSight = GetPointsInSight(params, rrt_conf(:,2:3), inspectionPoints, obstacles);
            clusters = InspectionClustering(params, rrt_conf(:,2:3), pointsInSight);
    end
%     PlotEnvironment(params, rrt_conf(:,2:3), clusters, M, inspectionPoints, obstacles, 'Spectral');
%     title(num2str(i));
    
    [bridge_conf, bridge_vertex, bridge_edges, vert_idx_mapping, vedges_mapping] = get_bridge_graph(rrt_conf, rrt_vertex, rrt_edges, clusters, obstacles, params, true);
    
%     M = Edges2M(bridge_edges(:, [1:2, 7]));
%     clusters = ones(size(bridge_conf, 1), 1);
%     PlotEnvironment(params, bridge_conf(:,2:3), clusters, M, inspectionPoints, obstacles, 'Spectral');
    
    virtual_edges = cell2mat(vedges_mapping(:, 1));

%     scatter(bridge_conf(:, 2), bridge_conf(:, 3), 'ok', 'Linewidth', 1);
%     for k = 1:size(bridge_edges, 1)
%         p1 = bridge_conf(bridge_edges(k, 1)+1, 2:3);
%         p2 = bridge_conf(bridge_edges(k, 2)+1, 2:3);
%         vEdgeIdx = find(virtual_edges == k);
%         if isempty(vEdgeIdx)
%             plot([p1(1) p2(1)], [p1(2) p2(2)], '--r', 'LineWidth', 2);
%         else
%             % a virtual edge
%             plot([p1(1) p2(1)], [p1(2) p2(2)], '-.r', 'LineWidth', 2);
%             % show the real path
%             vEdgeStartRealIdx = vert_idx_mapping(bridge_edges(k, 1)+1);
%             vEdgeEndRealIdx = vert_idx_mapping(bridge_edges(k, 2)+1);
%             vertInRealPath = vedges_mapping{vEdgeIdx, 2}';
%             edgesInRealPath = get_edges_per_vertex(rrt_edges, [vEdgeStartRealIdx, vEdgeEndRealIdx, vertInRealPath]);
%             for kk = 1:size(edgesInRealPath, 1)
%                 p1 = rrt_conf(edgesInRealPath(kk, 1)+1, 2:3);
%                 p2 = rrt_conf(edgesInRealPath(kk, 2)+1, 2:3);
%                 plot([p1(1) p2(1)], [p1(2) p2(2)], '-.b', 'LineWidth', 2);
%             end
%         end
%     end

    write_graph(fullfile(base_name, [env_name '_bridges'],['bridge_' num2str(i)]), bridge_conf, bridge_vertex, unique(bridge_edges, 'rows'));
end

% Show the last bridge graph
M = Edges2M(rrt_edges(:, [1:2,7]));
switch clusteringMethod
    case 'kmeans'
        clusters = KMeansClustering(params, rrt_conf(:,2:3), num_rooms);
    case 'spectral'
        clusters = SpectralClustering(params, rrt_conf(:,2:3), M);
    case 'inspection'
        pointsInSight = GetPointsInSight(params, rrt_conf(:,2:3), inspectionPoints, obstacles);
        clusters = InspectionClustering(params, rrt_conf(:,2:3), pointsInSight);
end

PlotEnvironment(params, rrt_conf(:,2:3), clusters, M, inspectionPoints, obstacles, ['Clustered Environment Using ' clusteringMethod ' Clustering']);

PlotEnvironment(params, rrt_conf(:,2:3), clusters, M, inspectionPoints, obstacles, ['Original and Bridges Graphs after ' num2str(i) ' RRT Iterations']);
scatter(bridge_conf(:, 2), bridge_conf(:, 3), 'om', 'Linewidth', 1);
for k = 1:size(bridge_edges, 1)
    p1 = bridge_conf(bridge_edges(k, 1)+1, 2:3);
    p2 = bridge_conf(bridge_edges(k, 2)+1, 2:3);
    vEdgeIdx = find(virtual_edges == k);
    if isempty(vEdgeIdx)
        plot([p1(1) p2(1)], [p1(2) p2(2)], '--r', 'LineWidth', 2);
    else
        % a virtual edge
        plot([p1(1) p2(1)], [p1(2) p2(2)], '-.r', 'LineWidth', 2);
        % show the real path
        vEdgeStartRealIdx = vert_idx_mapping(bridge_edges(k, 1)+1);
        vEdgeEndRealIdx = vert_idx_mapping(bridge_edges(k, 2)+1);
        vertInRealPath = vedges_mapping{vEdgeIdx, 2}';
        edgesInRealPath = get_edges_per_vertex(rrt_edges, [vEdgeStartRealIdx, vEdgeEndRealIdx, vertInRealPath]);
        for kk = 1:size(edgesInRealPath, 1)
            p1 = rrt_conf(edgesInRealPath(kk, 1)+1, 2:3);
            p2 = rrt_conf(edgesInRealPath(kk, 2)+1, 2:3);
            plot([p1(1) p2(1)], [p1(2) p2(2)], '-.b', 'LineWidth', 2);
        end
    end
end

% plotGraph(bridge_conf, bridge_vertex, bridge_edges);
% title('Bridge graph AFTER inner edges');

% % show the bridge graph
% PlotEnvironment(params, conf(:,2:3), clusters, M, inspectionPoints, obstacles, 'Spectral');
% % scatter(bridge_conf(:, 2), bridge_conf(:, 3), 'x', 'Linewidth', 1);
% ubridge_edges = unique(bridge_edges, 'rows');
% for k = 1:size(ubridge_edges, 1)
%     p1 = conf(ubridge_edges(k, 1)+1, 2:3);
%     p2 = conf(ubridge_edges(k, 2)+1, 2:3);
%     plot([p1(1) p2(1)], [p1(2) p2(2)], '--k')
% end

% edges_ = get_edges_per_vertex(bridge_edges, bridge_vertex(1:23, 1));
% PlotEnvironment(params, bridge_conf(1:23,2:3), clusters(1:23), M, inspectionPoints, obstacles, 'Spectral');
% for k = 1:size(edges_, 1)
%     p1 = conf(edges_(k, 1)+1, 2:3);
%     p2 = conf(edges_(k, 2)+1, 2:3);
%     plot([p1(1) p2(1)], [p1(2) p2(2)], '--k')
% end

% env_name = [env_name '_bridge'];
% write_graph(fullfile(base_name, env_name), bridge_conf, bridge_vertex, unique(bridge_edges, 'rows'));
% Graph2Text(params, fullfile(base_name, env_name), bridge_conf(:, 2:3), pointsInSight, M);

% if PLOT_ENVIRONMENMT && ~isempty(obstacles)
%     PlotEnvironment(params, conf(:,2:3), clusters, M, inspectionPoints, obstacles, clusteringMethod);
% end

if RUN_SEARCH
    % Run in WSL
    file_to_read = [base_name_in_wsl '/' env_name];
    file_to_write = [base_name_in_wsl '/' env_name];
    file_to_write_bridge = [base_name_in_wsl '/' env_name '_bridge'];
    cmd = [
        'wsl ' ...
        search_path ' ' ...
        file_to_read ' ' ...
        num2str(initial_p) ' ' ...
        num2str(initial_eps) ' ' ...
        num2str(tightening_rate) ' ' ...
        num2str(method) ' ' ...
        file_to_write ' ' ...
        '1 0'];
    cmd_bridge = [
        'wsl ' ...
        search_path ' ' ...
        file_to_read ' ' ...
        num2str(initial_p) ' ' ...
        num2str(initial_eps) ' ' ...
        num2str(tightening_rate) ' ' ...
        num2str(method) ' ' ...
        file_to_write_bridge ' ' ...
        num2str(step_in_bridge_search) ' 1'];
    disp('Executing command on WSL: ');
    disp(cmd);
    status = system(cmd);
    if status
        error('Failed to run');
    end
    status = system(cmd_bridge);
    if status
        error('Failed to run bridges');
    end
end

% Read result
res_file = [base_name '\' env_name '_result'];
res_file_bridges = [base_name '\' env_name '_bridge_result'];
if exist(res_file, 'file') && exist(res_file_bridges, 'file')
    fid = fopen(res_file);
    C = textscan(fid, '%s', 'delimiter','\n');
    fclose(fid);
    C = C{1, 1};
    out = C{end};
    outsplt = strsplit(strtrim(out), ' ');
    pathIdx = str2double(outsplt(2:end)) + 1;
    
    fid = fopen(res_file_bridges);
    C = textscan(fid, '%s', 'delimiter','\n');
    fclose(fid);
    C = C{1, 1};
    out = C{end};
    outsplt = strsplit(strtrim(out), ' ');
    pathIdxBridges = str2double(outsplt(2:end)) + 1;
    
    PlotEnvironment(params, rrt_conf(:,2:3), clusters, M, inspectionPoints, obstacles, 'Paths found by IRIS');
    % Show the original path
    addToLegend = [];
    for i = 1:length(pathIdx) - 1
        h = plot([conf(pathIdx(i), 2), conf(pathIdx(i+1), 2)], ...
            [conf(pathIdx(i), 3), conf(pathIdx(i+1), 3)], ...
            'g', 'Marker', 'o', 'LineWidth', 1, 'DisplayName', 'Original');
        if i == 1
            h.DisplayName = 'Original Path';
            addToLegend = [addToLegend h];
        end
    end
    
    % plot bridges path
    % TODO currently taking the last rrt iteration
    [bridge_conf, bridge_vertex, bridge_edges, vert_idx_mapping, vedges_mapping] = get_bridge_graph(rrt_conf, rrt_vertex, rrt_edges, clusters, obstacles, params, true);
    virtual_edges = cell2mat(vedges_mapping(:, 1));
    
    vertInBridgePath = bridge_vertex(pathIdxBridges, 1);

    % show bridge vertices
    scatter(bridge_conf(:, 2), bridge_conf(:, 3), 'om', 'Linewidth', 1);
%     realVertIndInBridgePath = vert_idx_mapping(pathIdxBridges);
%     pathRealIdxBridges = realVertIndInBridgePath + 1;
    pathRealIdxBridges = [];
    
    for i = 1:length(pathIdxBridges) - 1
        p1 = pathIdxBridges(i);
        p2 = pathIdxBridges(i + 1);
        [e, idx] = get_edges_per_vertex(bridge_edges, [p1 - 1, p2 - 1]);
        vEdgeIdx = find(virtual_edges == idx);
        pathRealIdxBridges = [pathRealIdxBridges vert_idx_mapping(p1)];
        if ~isempty(vEdgeIdx)
            % this is a virtual edge, get it's mapping
            innerInd = vedges_mapping{vEdgeIdx, 2}';
            pathRealIdxBridges = [pathRealIdxBridges innerInd];
        end
    end
    pathRealIdxBridges = [pathRealIdxBridges vert_idx_mapping(p2)];
    
    pathRealIdxBridges = pathRealIdxBridges + 1; % convert to matlab ind
    
    for i = 1:length(pathRealIdxBridges) - 1
        h = plot([rrt_conf(pathRealIdxBridges(i), 2), rrt_conf(pathRealIdxBridges(i+1), 2)], ...
            [rrt_conf(pathRealIdxBridges(i), 3), rrt_conf(pathRealIdxBridges(i+1), 3)], ...
            'r', 'Marker', 'x', 'LineWidth', 1, 'DisplayName', 'Bridges', 'LineStyle','--');
        if i == 1
            h.DisplayName = 'Path Using Bridge Graph';
            addToLegend = [addToLegend h];
        end
    end
    
    % calculate coverage set
    cov_set1 = [];
    for i = 1:length(pathIdx)
        cov = vertex(pathIdx(i), 5:end);
        cov_set1 = [cov_set1 cov(~isnan(cov))];
    end
    cov_set1 = unique(cov_set1) + 1;
    scatter(inspectionPoints(cov_set1, 1), inspectionPoints(cov_set1, 2), 'og')
    
    cov_set2 = [];
    for i = 1:length(pathRealIdxBridges)
        cov = vertex(pathRealIdxBridges(i), 5:end);
        cov_set2 = [cov_set2 cov(~isnan(cov))];
    end
    cov_set2 = unique(cov_set2) + 1;
    scatter(inspectionPoints(cov_set2, 1), inspectionPoints(cov_set2, 2), 'xr')
    
    addToLegend(1).DisplayName = [addToLegend(1).DisplayName ' (' ...
        num2str(roundn(length(cov_set1)/size(inspectionPoints, 1),-1)*100) '% Coverage)'];
    
    addToLegend(2).DisplayName = [addToLegend(2).DisplayName ' (' ...
        num2str(roundn(length(cov_set2)/size(inspectionPoints, 1),-1)*100) '% Coverage)'];
    legend(addToLegend, 'Location', 'BestOutside');
end

return;

% Show the graph
% s = edges(:, 1) + 1;
% t = edges(:, 2) + 1;
% G = graph(s,t);
% figure;
% h = plot(G);

% % Define distance metric for configuration
% conf_dist = @(c1, c2) norm(c1 - c2);
% 
% % Define the data for clustering (configuration data)
% conf_data = conf(:, 2:end);
% 
% all_inp_pts = vertex(:, 4:end);
% all_inp_pts = all_inp_pts(:);
% all_inp_pts = all_inp_pts(~isnan(all_inp_pts));
% total_inspection_points = max(all_inp_pts); % TODO
% 
% % perform k-means
% f = figure;
% c = colormap('hsv');
% close(f);
% 
% c = c(randperm(size(c, 1)), :);
% 
% diff_vec = zeros(1, length(k_vec));
% for k_idx = 1:length(k_vec)
%     k = k_vec(k_idx);
%     
%     M = Edges2M(edges);
%     switch clusteringMethod
%         case 'kmeans'
%             clusters = KMeansClustering(params, conf(:,2:3), k);
%         case 'spectral'
%             clusters = SpectralClustering(params, conf(:,2:3), M, k);
%         case 'inspection'
%             warning('Inspection Clustering is not relevant here')
%             break
%     end
%     
%     if PLOT_CLUSTERING
%        PlotEnvironment(params, conf_data, clusters, M, inspectionPoints, obstacles, ['k=', num2str(k)]);
%     end
%     
%     % Check which coverage we have in each cluster
%     cov_set_per_cluster = cell(k, 1);
%     if PLOT_HISTOGRAMS
%         figure;
%     end
%     for i = 1:k
%         cl_vertex_idx = idx == i;
%         cl_vertex = vertex(cl_vertex_idx, :);
%         cl_vertex_cov = vertex(cl_vertex_idx, 4:end);
%         cl_vertex_cov_col = cl_vertex_cov(:);
%         cl_vertex_cov_col = cl_vertex_cov_col(~isnan(cl_vertex_cov_col));
%         %         figure; histogram(cl_vertex_cov_col, unique(cl_vertex_cov_col))
%         [upts, ia, ic] = unique(cl_vertex_cov_col);
%         num_inspection_points = length(upts);
%         N = arrayfun(@(x) sum(cl_vertex_cov_col == x), upts);
%         %         h = histcounts(cl_vertex_cov_col, unique(cl_vertex_cov_col));
%         %         N = h.BinCounts;
%         maxN = max(N);
%         distb = N ./ maxN;
%         
%         cov_set_per_cluster{i, 1} = upts(distb > CVG_TH);
%         if PLOT_CLUSTERING
%             plot(inspectionPoints(cov_set_per_cluster{i},1), inspectionPoints(cov_set_per_cluster{i},2), 'sq')
%         end
%         if PLOT_HISTOGRAMS
%             plot(upts, N, 'DisplayName', ['Cluster ' num2str(i)], 'Color', c(i, :), 'LineStyle', 'None', 'Marker', 'o');
%             hold on;
%         end
%     end
%     if PLOT_HISTOGRAMS
%         title(['K=' num2str(k)]);
%     end
%     %     legend('show');
%     sim_mat = ones(k, k)*nan;
%     
%     for i = 1:k
%         for j = i+1:k
%             ints = intersect(cov_set_per_cluster{i}, cov_set_per_cluster{j});
%             unio = union(cov_set_per_cluster{i}, cov_set_per_cluster{j});
%             jaccard_dist = length(ints) / length(unio);
%             sim_mat(i, j) = length(ints) / min(numel(cov_set_per_cluster{i}), numel(cov_set_per_cluster{j}));
%         end
%     end
%     
%     sim_values = sim_mat(~isnan(sim_mat));
%     diff_measure = mean(sim_values);
%     diff_vec(k_idx) = diff_measure;
%     %disp(['K=' num2str(k) ', Diff=' num2str(diff_measure)]);
% end
% 
% figure;
% plot(k_vec, diff_vec, '.-');
