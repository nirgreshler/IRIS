close all
clc
clear

% env_name = 'planar_1000';
env_name = 'syn_9rooms';

K_MEANS_START = 2;
K_MEANS_END = 10;
k_vec = K_MEANS_START:K_MEANS_END;
CVG_TH = 0.8;
addpath(genpath(pwd))

SPECTRAL = true;

% rng(K_MEANS_START);

% PLOTTING
PLOT_ENVIRONMENMT = false;
PLOT_CLUSTERING = true;
PLOT_HISTOGRAMS = false;

% SOLVING
RUN_SEARCH = true;
initial_p = 0.99;
initial_eps = 0.8;
tightening_rate = 0;
method = 0;

dist = 'sqeuclidean';
base_name = fullfile(pwd, 'Graphs');
wsl_path = '/home/nirgreshler/Project/IRIS';
search_path = [wsl_path, '/debug/app/search_graph'];
base_name_in_wsl = ['/mnt/' lower(strrep(strrep(base_name,':',''),'\','/'))];

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));
M = Edges2M(edges(:,[1 2 7]));
if PLOT_ENVIRONMENMT && ~isempty(obstacles)
    [clustersKmeans, clustersSpectral] = ClusterPoints(conf(:,2:3), Edges2M(edges));
    PlotEnvironment(conf(:,2:3), clustersKmeans, M, inspectionPoints, obstacles, params.homeSize);
end

if RUN_SEARCH
    % Run in WSL
    file_to_read = [base_name_in_wsl '/' env_name];
    file_to_write = [base_name_in_wsl '/' env_name];
    status = system([
        'wsl ' ...
        search_path ' ' ...
        file_to_read ' ' ...
        num2str(initial_p) ' ' ...
        num2str(initial_eps) ' ' ...
        num2str(tightening_rate) ' ' ...
        num2str(method) ' ' ...
        file_to_write]);
    if status
        error('Failed to run');
    end
    
    % Read result
    res_file = [base_name '\' env_name '_result'];
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
    [clustersKmeans, clustersSpectral] = ClusterPoints(conf(:,2:3), Edges2M(edges));
    % plot enviroment
    PlotEnvironment(conf(:,2:3), clustersKmeans, M, inspectionPoints, obstacles, params.homeSize);
    % plot path
    for i = 1:length(pathIdx) - 1
        plot([conf(pathIdx(i), 2), conf(pathIdx(i+1), 2)], [conf(pathIdx(i), 3), conf(pathIdx(i+1), 3)], ...
            'g', 'Marker', 'o', 'LineWidth', 2)
    end
end
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
    
    [clustersKmeans, clustersSpectral] = ClusterPoints(conf_data, Edges2M(edges), k);
    
    if SPECTRAL
        idx = clustersSpectral;
    else
        idx = clustersKmeans;
%         idx = kmeans(conf_data, k, 'Distance', dist);
    end

    if PLOT_CLUSTERING
       PlotEnvironment(conf_data, idx, M, inspectionPoints, obstacles, params.homeSize, ['k=', num2str(k)]);
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
