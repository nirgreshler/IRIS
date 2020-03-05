% close all
clc
clear

% env_name = 'planar_1000';
num_rooms = 9;
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
wsl_path = ['/home/' username '/Project/IRIS'];
search_path = [wsl_path, '/debug/app/search_graph'];
base_name_in_wsl = ['/mnt/' lower(strrep(strrep(base_name,':',''),'\','/'))];

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));

% Perform clustering
M = Edges2M(edges(:,[1 2 7]));
switch clusteringMethod
    case 'kmeans'
        clusters = KMeansClustering(params, conf(:,2:3), num_rooms);
    case 'spectral'
        clusters = SpectralClustering(params, conf(:,2:3), M, num_rooms);
    case 'inspection'
        pointsInSight = GetPointsInSight(params, conf(:,2:3), inspectionPoints, obstacles);
        clusters = InspectionClustering(params, conf(:,2:3), pointsInSight);
end

% build the bridge graph
[bridge_conf, bridge_vertex, bridge_edges] = get_bridge_graph(conf, vertex, edges, clusters);

% add the inner edges within a cluster
edges_added = 0;
numClusters = length(unique(clusters));
for iClus = 1:numClusters
    vertInClus = vertex(clusters == iClus, 1);  
    vertInClusInBridge = intersect(vertInClus, bridge_vertex(:, 1));  
    points = conf(vertInClus + 1, 2:3);
    Mc = BuildAdjcancyMatrix(points, obstacles, params.connectionRadius);
    for i = 1:length(vertInClusInBridge)-1
        for j = i+1:length(vertInClusInBridge)
            startIdx = find(all(points == conf(vertInClusInBridge(i) + 1, 2:3),2));
            goalIdx = find(all(points == conf(vertInClusInBridge(j) + 1, 2:3),2));
            pathFound = AStar(points, startIdx, goalIdx, Mc);
            if ~isempty(pathFound)
                cost = sum(sqrt(sum(diff(pathFound(:,2:3)).^2,2)));
                newEdge = [vertInClusInBridge(i), vertInClusInBridge(j), 1, 1, 0, 0, cost]; % TODO 1 1 0 0
                bridge_edges = [bridge_edges; newEdge];
                edges_added = edges_added + 1;
            end
        end
    end
end
disp(['Added ' num2str(edges_added) ' edges within clusters']);

% show the bridge graph
PlotEnvironment(params, conf(:,2:3), clusters, M, inspectionPoints, obstacles, 'Spectral');
% scatter(bridge_conf(:, 2), bridge_conf(:, 3), 'x', 'Linewidth', 1);
ubridge_edges = unique(bridge_edges, 'rows');
for k = 1:size(ubridge_edges, 1)
    p1 = conf(ubridge_edges(k, 1)+1, 2:3);
    p2 = conf(ubridge_edges(k, 2)+1, 2:3);
    plot([p1(1) p2(1)], [p1(2) p2(2)], '--k')
end

% need to fix indices
nNodes = size(bridge_vertex(:, 1), 1);
idx_in_edges = bridge_vertex(:, 1);
bridge_vertex(:, 1) = 0:nNodes-1;
bridge_conf(:, 1) = 0:nNodes-1;
edgesIdx = bridge_edges(:, 1:2);
for i = 0:nNodes-1
    edgesIdx(edgesIdx == idx_in_edges(i+1)) = i;
end
bridge_edges(:, 1:2) = edgesIdx;

env_name = [env_name '_bridge'];
write_graph(fullfile(base_name, env_name), bridge_conf, bridge_vertex, unique(bridge_edges, 'rows'));
% Graph2Text(params, fullfile(base_name, env_name), bridge_conf(:, 2:3), pointsInSight, M);

if PLOT_ENVIRONMENMT && ~isempty(obstacles)
    PlotEnvironment(params, conf(:,2:3), clusters, M, inspectionPoints, obstacles, clusteringMethod);
end

if RUN_SEARCH
    % Run in WSL
    file_to_read = [base_name_in_wsl '/' env_name];
    file_to_write = [base_name_in_wsl '/' env_name];
    cmd = [
        'wsl ' ...
        search_path ' ' ...
        file_to_read ' ' ...
        num2str(initial_p) ' ' ...
        num2str(initial_eps) ' ' ...
        num2str(tightening_rate) ' ' ...
        num2str(method) ' ' ...
        file_to_write];
    disp('Executing command on WSL: ');
    disp(cmd);
    status = system(cmd);
    if status
        error('Failed to run');
    end
end

% Read result
res_file = [base_name '\' env_name '_result'];
if exist(res_file, 'file')
    fid = fopen(res_file);
    C = textscan(fid, '%s', 'delimiter','\n');
    fclose(fid);
    C = C{1, 1};
    %     [status, output] = system(['wsl cat ' res_file]);
    %     if status
    %         error('Failed to read output');
    %     end
    
    out = C{end};
    %     splt = strsplit(output, '\n');
    %     out = splt{end - 1};
    outsplt = strsplit(strtrim(out), ' ');
    pathIdx = str2double(outsplt(2:end)) + 1;
    
    % cluster
    switch clusteringMethod
        case 'kmeans'
            clusters = KMeansClustering(params, conf(:,2:3), num_rooms);
        case 'spectral'
            clusters = SpectralClustering(params, conf(:,2:3), M, num_rooms);
        case 'inspection'
            pointsInSight = GetPointsInSight(params, conf(:,2:3), inspectionPoints, obstacles);
            clusters = InspectionClustering(params, conf(:,2:3), pointsInSight);
    end
    % plot enviroment
    PlotEnvironment(params, conf(:,2:3), clusters, M, inspectionPoints, obstacles);
    % plot path
    for i = 1:length(pathIdx) - 1
        plot([conf(pathIdx(i), 2), conf(pathIdx(i+1), 2)], [conf(pathIdx(i), 3), conf(pathIdx(i+1), 3)], ...
            'g', 'Marker', 'o', 'LineWidth', 2)
    end
    
    % calculate coverage set
    cov_set = [];
    for i = 1:length(pathIdx)
        cov = vertex(pathIdx(i), 5:end);
        cov_set = [cov_set cov(~isnan(cov))];
    end
    cov_set = unique(cov_set) + 1;
    scatter(inspectionPoints(cov_set, 1), inspectionPoints(cov_set, 2), '*')
end

return;

% Show the graph
% s = edges(:, 1) + 1;
% t = edges(:, 2) + 1;
% G = graph(s,t);
% figure;
% h = plot(G);

% Define distance metric for configuration
conf_dist = @(c1, c2) norm(c1 - c2);

% Define the data for clustering (configuration data)
conf_data = conf(:, 2:end);

all_inp_pts = vertex(:, 4:end);
all_inp_pts = all_inp_pts(:);
all_inp_pts = all_inp_pts(~isnan(all_inp_pts));
total_inspection_points = max(all_inp_pts); % TODO

% perform k-means
f = figure;
c = colormap('hsv');
close(f);

c = c(randperm(size(c, 1)), :);

diff_vec = zeros(1, length(k_vec));
for k_idx = 1:length(k_vec)
    k = k_vec(k_idx);
    
    M = Edges2M(edges);
    switch clusteringMethod
        case 'kmeans'
            clusters = KMeansClustering(params, conf(:,2:3), k);
        case 'spectral'
            clusters = SpectralClustering(params, conf(:,2:3), M, k);
        case 'inspection'
            warning('Inspection Clustering is not relevant here')
            break
    end
    
    if PLOT_CLUSTERING
       PlotEnvironment(params, conf_data, clusters, M, inspectionPoints, obstacles, ['k=', num2str(k)]);
    end
    
    % Check which coverage we have in each cluster
    cov_set_per_cluster = cell(k, 1);
    if PLOT_HISTOGRAMS
        figure;
    end
    for i = 1:k
        cl_vertex_idx = idx == i;
        cl_vertex = vertex(cl_vertex_idx, :);
        cl_vertex_cov = vertex(cl_vertex_idx, 4:end);
        cl_vertex_cov_col = cl_vertex_cov(:);
        cl_vertex_cov_col = cl_vertex_cov_col(~isnan(cl_vertex_cov_col));
        %         figure; histogram(cl_vertex_cov_col, unique(cl_vertex_cov_col))
        [upts, ia, ic] = unique(cl_vertex_cov_col);
        num_inspection_points = length(upts);
        N = arrayfun(@(x) sum(cl_vertex_cov_col == x), upts);
        %         h = histcounts(cl_vertex_cov_col, unique(cl_vertex_cov_col));
        %         N = h.BinCounts;
        maxN = max(N);
        distb = N ./ maxN;
        
        cov_set_per_cluster{i, 1} = upts(distb > CVG_TH);
        if PLOT_CLUSTERING
            plot(inspectionPoints(cov_set_per_cluster{i},1), inspectionPoints(cov_set_per_cluster{i},2), 'sq')
        end
        if PLOT_HISTOGRAMS
            plot(upts, N, 'DisplayName', ['Cluster ' num2str(i)], 'Color', c(i, :), 'LineStyle', 'None', 'Marker', 'o');
            hold on;
        end
    end
    if PLOT_HISTOGRAMS
        title(['K=' num2str(k)]);
    end
    %     legend('show');
    sim_mat = ones(k, k)*nan;
    
    for i = 1:k
        for j = i+1:k
            ints = intersect(cov_set_per_cluster{i}, cov_set_per_cluster{j});
            unio = union(cov_set_per_cluster{i}, cov_set_per_cluster{j});
            jaccard_dist = length(ints) / length(unio);
            sim_mat(i, j) = length(ints) / min(numel(cov_set_per_cluster{i}), numel(cov_set_per_cluster{j}));
        end
    end
    
    sim_values = sim_mat(~isnan(sim_mat));
    diff_measure = mean(sim_values);
    diff_vec(k_idx) = diff_measure;
    %disp(['K=' num2str(k) ', Diff=' num2str(diff_measure)]);
end

figure;
plot(k_vec, diff_vec, '.-');
