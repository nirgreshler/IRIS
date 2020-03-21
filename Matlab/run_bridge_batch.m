%% Init
close all
clc
clear
rng(1)
addpath(genpath(pwd))
RUN_ORIGINAL = false;

%% Path settings
define_path;

%% Environment settings
env_name = 'drone_big';
n_vertices = 2000;
filename = [env_name, '_', num2str(n_vertices)];
search_path = [wsl_path, '/', 'build_', env_name, '/app/search_graph'];
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, filename));

%% Plotting settings
params.showText = false;
params.plotEdges = false;

%% Clustering settings
clusteringMethod = 'spectral'; % 'kmeans' / 'spectral' / 'inspection'
params.maxClusters = 20;
params.minClusters = 10;
params.useExpDist = false;
params.inspectionPoints = inspectionPoints;
params.obstacles = obstacles;
% create clustering object
clustering = Clustering(clusteringMethod, params);

%% IRIS settings
initial_p_orig = '0.25';
initial_eps = '0.5';
tightening_rate = '0';
method = '0';

%% Algorithm settings
USE_VIRTUAL_VERTICES = false;
RUN_IRIS_IN_CLUSTERS = false;
IRIS_IN_CLUSTER_COV_TH = 0.5;
%% Create the graphs
original_graph_path = fullfile(base_name, filename);
bridge_graph_path = fullfile(base_name, [filename '_bridge']);
% read original graph
G = IGraph(original_graph_path, clustering);
nClusters = length(unique(G.graph.Nodes.cluster));
fprintf('Clustered to %d clusters\n', nClusters)

file_to_read = [base_name_in_wsl '/' filename];
file_to_write = [base_name_in_wsl '/' filename];
cmd = {'wsl', search_path, file_to_read, ...
    initial_p_orig, initial_eps, tightening_rate, method, ...
    file_to_write, num2str(G.num_vertices()), '0'};

if RUN_ORIGINAL
    [pathId, runtime_original] = G.run_search(cmd);
    cov_set = calc_coverage(G, pathId);
    cost_orig = calc_cost(G, pathId);
    origCoverage = length(cov_set);
else
    origCoverage = 18487;
    cost_orig = 0.18;
    runtime_original = 1157.3;
end

minNumBridgesVec = 25;
pVec = 0.4;
covVec = zeros(length(pVec), length(minNumBridgesVec));
costVec = zeros(length(pVec), length(minNumBridgesVec));
runtimeVec = zeros(length(pVec), length(minNumBridgesVec));
nVerticesVec = zeros(length(pVec), length(minNumBridgesVec));
nEdgesVec = zeros(length(pVec), length(minNumBridgesVec));
for pIdx = 1:length(pVec)
    initial_p = num2str(pVec(pIdx));
    for nIdx = 1:length(minNumBridgesVec)
        fprintf('Running %d out of %d...\n', length(minNumBridgesVec)*(pIdx-1)+nIdx, length(pVec)*length(minNumBridgesVec))
        minNumBridges = minNumBridgesVec(nIdx);
        BG = G.build_bridge_graph(USE_VIRTUAL_VERTICES, minNumBridges);
        BG.write_graph(bridge_graph_path);
        % Run the search in WSL
        file_to_read = [base_name_in_wsl '/' filename];
        file_to_write = [base_name_in_wsl '/' filename];
        
        cmd = {'wsl', search_path, file_to_read, ...
            initial_p, initial_eps, tightening_rate, method, ...
            file_to_write, num2str(G.num_vertices()), '0'};
        
        % run bridge
        cmd{3} = [cmd{3} '_bridge'];
        cmd{8} = [cmd{8} '_bridge'];
        cmd{9} = num2str(BG.num_vertices());
        [pathIdBridge, runtime_bridge] = BG.run_search(cmd);
        realPathId = extract_real_path(G, BG, pathIdBridge);
        cov_set_bridge = calc_coverage(G, realPathId);
        cost_bridge = calc_cost(G, realPathId);
        covVec(pIdx, nIdx) = length(cov_set_bridge);
        costVec(pIdx, nIdx) = cost_bridge;
        runtimeVec(pIdx, nIdx) = runtime_bridge;
        nVerticesVec(pIdx, nIdx) = size(BG.graph.Nodes,1);
        nEdgesVec(pIdx, nIdx) = size(BG.graph.Edges,1);
    end
end
%%
figure; plot(covVec(:), '.-')
hold all; plot([1 length(pVec)*length(minNumBridgesVec)], [origCoverage origCoverage], '--k')
title('Coverage')

figure; plot(costVec(:), '.-')
hold all; plot([1 length(pVec)*length(minNumBridgesVec)], [cost_orig cost_orig], '--k')
title('Cost')

figure; plot(runtimeVec(:), '.-')
hold all; plot([1 length(pVec)*length(minNumBridgesVec)], [runtime_original runtime_original], '--k')
title('Runtime')

fprintf('Original:\n%.2f\n%d\n%d\n%d\n%.2f\n%.1f\n',...
    str2num(initial_p_orig), size(G.graph.Nodes,1), size(G.graph.Edges,1), origCoverage, cost_orig, runtime_original)
%%
chosenIdx = 7;
[i,j] = ind2sub([length(pVec), length(minNumBridgesVec)], chosenIdx);
chosenP = pVec(i);
chosenMinNumBridge = minNumBridgesVec(j);
nVertices = nVerticesVec(i,j);
nEdges = nEdgesVec(i,j);
coverage = covVec(i,j);
cost = costVec(i,j);
runtime = runtimeVec(i,j);
clc
fprintf('Bridges:\n%d\n%.2f\n%d\n%d\n%d\n%.2f\n%.1f\n',...
    chosenMinNumBridge, chosenP, nVertices, nEdges, coverage, cost, runtime)