%% Init
close all
clc
clear
rng(1)
addpath(genpath(pwd))

%% Path settings
define_path;

%% Environment settings
num_rooms = 4;
env_name = ['syn_' num2str(num_rooms) 'rooms'];
% env_name = 'crisp_100';
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));

%% Plotting settings
params.showText = false;
params.plotEdges = true;

%% Clustering settings
clusteringMethod = 'spectral'; % 'kmeans' / 'spectral' / 'inspection'
params.maxClusters = 10;
params.minClusters = 4;
% create clustering object
clustering = Clustering(clusteringMethod, params);

%% IRIS settings
initial_p = '0.8';
initial_eps = '0.8';
tightening_rate = '0';
method = '0';

%% Create the graphs
original_graph_path = fullfile(base_name, env_name);
bridge_graph_path = fullfile(base_name, [env_name '_bridge']);
% read original graph
G = IGraph(original_graph_path, clustering);
% build bridge graph and write to file
tic
BG = G.build_bridge_graph();
build_bridge_time = toc;
BG.write_graph(bridge_graph_path);

%% Show the graphs
PlotEnvironment(params, G, inspectionPoints, obstacles, 'Original Clustered Graph');
scatter(BG.graph.Nodes.x1, BG.graph.Nodes.x2, 'om', 'Linewidth', 1);
PlotEnvironment(params, BG, inspectionPoints, obstacles, 'Bridge Graph');

%% Run the search in WSL
file_to_read = [base_name_in_wsl '/' env_name];
file_to_write = [base_name_in_wsl '/' env_name];
cmd = {'wsl', search_path, file_to_read, ...
    initial_p, initial_eps, tightening_rate, method, ...
    file_to_write, num2str(G.num_vertices), '0'};

% run original
tic
status = system(strjoin(cmd));
runtime_original = toc;
if status
    error('Failed to run original');
end

% run bridge
cmd{3} = [cmd{3} '_bridge'];
cmd{8} = [cmd{8} '_bridge'];
cmd{9} = num2str(BG.num_vertices());
tic
status = system(strjoin(cmd));
runtime_bridge = toc;
if status
    error('Failed to run bridge');
end

%% Read results
res_file = [base_name '\' env_name '_result'];
res_file_bridge = [base_name '\' env_name '_bridge_result'];

pathId = read_result(res_file);
pathIdBridge = read_result(res_file_bridge);

%% Show path
PlotEnvironment(params, G, inspectionPoints, obstacles, 'Paths found by IRIS');
scatter(BG.graph.Nodes.x1, BG.graph.Nodes.x2, 'om', 'Linewidth', 1);
% show the original path
h = show_path(G, pathId, 'go-');
h.DisplayName = 'Original Path';
addToLegend = h;

% show the bridge path
realPathId = extract_real_path(G, BG, pathIdBridge);
h = show_path(G, realPathId, 'rx-.');
h.DisplayName = 'Path Using Bridge Graph';
addToLegend = [addToLegend h];

%% Calc coverage
cov_set = calc_coverage(G, pathId);
cov_set_bridge = calc_coverage(G, realPathId);
% show coverage
scatter(inspectionPoints(cov_set, 1), inspectionPoints(cov_set, 2), 'og');
scatter(inspectionPoints(cov_set_bridge, 1), inspectionPoints(cov_set_bridge, 2), 'xr');
% add covergae to legend
addToLegend(1).DisplayName = [addToLegend(1).DisplayName ' (' ...
    num2str(roundn(length(cov_set)/size(inspectionPoints, 1),-1)*100) '% Coverage)'];

addToLegend(2).DisplayName = [addToLegend(2).DisplayName ' (' ...
    num2str(roundn(length(cov_set_bridge)/size(inspectionPoints, 1),-1)*100) '% Coverage)'];

legend(addToLegend, 'Location', 'BestOutside');

%% Show runtime
disp(['Original search runtime: ' num2str(runtime_original)]);
disp(['Bridge graph build time: ' num2str(build_bridge_time)]);
disp(['Bridge search runtime: ' num2str(runtime_bridge)]);