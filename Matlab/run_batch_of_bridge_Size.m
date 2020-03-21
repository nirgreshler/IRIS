%% Init
close all
clc
clear
rng(1)
addpath(genpath(pwd))
RUN_ORIGINAL = true;

%% Path settings
define_path;

%% Environment settings
env_name = 'drone_big';
n_vertices = 1000;
search_path = [wsl_path, '/', 'build_', env_name, '/app/search_graph'];
build_path = [wsl_path, '/', 'build_', env_name, '/app/build_graph'];
filename = [env_name, '_', num2str(n_vertices)];
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, filename));
%% Clustering settings
clusteringMethod = 'spectral'; % 'kmeans' / 'spectral' / 'inspection'
params.maxClusters = 20;
params.minClusters = 2;
params.inspectionPoints = inspectionPoints;
params.obstacles = obstacles;
% create clustering object
clustering = Clustering(clusteringMethod, params);

%% IRIS settings
initial_p = '0.9';
initial_eps = '0.5';
tightening_rate = '0';
method = '0';

%% Algorithm settings
USE_VIRTUAL_VERTICES = false;
RUN_IRIS_IN_CLUSTERS = false;
IRIS_IN_CLUSTER_COV_TH = 0.5;
minNumBridges = 10;
%% Create the graphs
original_graph_path = fullfile(base_name, filename);
bridge_graph_path = fullfile(base_name, [filename '_bridge']);
% read original graph
G = IGraph(original_graph_path, clustering);
nClusters = length(unique(G.graph.Nodes.cluster));
fprintf('Clustered to %d clusters\n', nClusters)
% build bridge graph and write to file
tic
BG = G.build_bridge_graph(USE_VIRTUAL_VERTICES, minNumBridges);
build_bridge_time = toc;
BG.write_graph(bridge_graph_path);

fprintf('Original Graph Size: %d points, %d edges\n', size(G.graph.Nodes,1), size(G.graph.Edges,1))
fprintf('Bridge Graph Size: %d points, %d edges\n', size(BG.graph.Nodes,1), size(BG.graph.Edges,1))
% run bridge
file_to_write = [base_name_in_wsl '/' filename];
cmd = {'wsl', search_path, file_to_write, ...
    initial_p, initial_eps, tightening_rate, method, ...
    file_to_write, num2str(G.num_vertices()), '0'};
cmd{3} = [cmd{3} '_bridge'];
cmd{8} = [cmd{8} '_bridge'];
cmd{9} = num2str(BG.num_vertices());
[pathIdBridge, runtime_bridge] = BG.run_search(cmd);
realPathId = extract_real_path(G, BG, pathIdBridge);
cov_set_bridge = calc_coverage(G, realPathId);
cost_bridge = calc_cost(G, realPathId);
%% Monte Carlo Run
% Build regular graph with the same size as the BG
new_filename = [env_name '_', num2str(BG.num_vertices())];
new_graph_path = fullfile(base_name, new_filename);
file_to_write = [base_name_in_wsl '/', new_filename];
build_cmd = {'wsl', build_path, '1', num2str(BG.num_vertices()), file_to_write};
search_cmd = {'wsl', search_path, file_to_write, ...
    initial_p, initial_eps, tightening_rate, method, ...
    file_to_write, num2str(BG.num_vertices()), '0'};
nExperiments = 100;
costVec = zeros(nExperiments,1);
coverageVec = zeros(nExperiments,1);
for runNum = 1:nExperiments
    fprintf('Exp #%d/%d...\n', runNum, nExperiments)
    % Create environement
    build_cmd{3} = num2str(runNum);
    [status, cmdout] = system(strjoin(build_cmd));
    if status
        error('Failed to build');
    end
    
    % Run on it
    Gtag = IGraph(new_graph_path);
    [pathId, runtime_original] = Gtag.run_search(search_cmd, true);
    cov_set = calc_coverage(Gtag, pathId);
    cost = calc_cost(Gtag, pathId);
    costVec(runNum) = cost;
    coverageVec(runNum) = length(cov_set);
end
meanCost = mean(costVec);
meanCoverage = mean(coverageVec);
%%
fprintf('Bridge coverage: %d points, cost: %.2f.\n', length(cov_set_bridge), cost_bridge)
fprintf('IRIS avg. coverage: %d points (%.1f%% of bridge), avg. cost: %.2f. (%.1f%% of bridge)\n',...
    round(meanCoverage), 100*round(meanCoverage)/length(cov_set_bridge), meanCost, 100*meanCost/cost_bridge)