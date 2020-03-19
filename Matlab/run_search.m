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
num_rooms = 9;
env_name = ['syn_' num2str(num_rooms) 'rooms'];
% env_name = 'drone_1000_big';
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));

%% Plotting settings
params.showText = false;
params.plotEdges = false;

%% Clustering settings
clusteringMethod = 'spectral'; % 'kmeans' / 'spectral' / 'inspection'
params.maxClusters = 20;
params.minClusters = 2;
% create clustering object
clustering = Clustering(clusteringMethod, params);

%% IRIS settings
initial_p = '0.8';
initial_eps = '0.5';
tightening_rate = '0';
method = '0';

%% Algorithm settings
USE_VIRTUAL_VERTICES = false;
RUN_IRIS_IN_CLUSTERS = false;
IRIS_IN_CLUSTER_COV_TH = 0.5;
minNumBridges = inf;
%% Create the graphs
original_graph_path = fullfile(base_name, env_name);
bridge_graph_path = fullfile(base_name, [env_name '_bridge']);
% read original graph
G = IGraph(original_graph_path, clustering);
% build bridge graph and write to file
tic
BG = G.build_bridge_graph(USE_VIRTUAL_VERTICES, minNumBridges);
build_bridge_time = toc;
BG.write_graph(bridge_graph_path);

%% Show the graphs
if contains(env_name, 'syn')
    PlotEnvironment(params, G, inspectionPoints, obstacles, 'Original Clustered Graph');
    scatter(BG.graph.Nodes.x1, BG.graph.Nodes.x2, 'om', 'Linewidth', 1);
    PlotEnvironment(params, BG, inspectionPoints, obstacles, 'Bridge Graph');
elseif contains(env_name, 'drone')
    if contains(env_name, 'big')
        inspectionPoints = read_bridge_model('big');
    else
        inspectionPoints = read_bridge_model();
    end
    PlotBridgeEnvironment(params, G, inspectionPoints, 'Original Clustered Graph');
    plot3(BG.graph.Nodes.x1, BG.graph.Nodes.x2,BG.graph.Nodes.x3,  'om', 'Linewidth', 1);
    PlotBridgeEnvironment(params, BG, inspectionPoints, 'Bridge Graph');
end
nClusters = length(unique(G.graph.Nodes.cluster));
fprintf('Clustered to %d clusters\n', nClusters)
fprintf('Original Graph Size: %d points, %d edges\n', size(G.graph.Nodes,1), size(G.graph.Edges,1))
fprintf('Bridge Graph Size: %d points, %d edges\n', size(BG.graph.Nodes,1), size(BG.graph.Edges,1))
%% Run the search in WSL
file_to_read = [base_name_in_wsl '/' env_name];
file_to_write = [base_name_in_wsl '/' env_name];
cmd = {'wsl', search_path, file_to_read, ...
    initial_p, initial_eps, tightening_rate, method, ...
    file_to_write, num2str(G.num_vertices), '0'};

if RUN_ORIGINAL
    [pathId, runtime_original] = G.run_search(cmd);
    cov_set = calc_coverage(G, pathId);
    cost_orig = calc_cost(G, pathId);
else
    pathId = [1 1];
    runtime_original = 0;
    cov_set = [];
    cost_orig = 0;
end

% run bridge
cmd{3} = [cmd{3} '_bridge'];
cmd{8} = [cmd{8} '_bridge'];
cmd{9} = num2str(BG.num_vertices());
[pathIdBridge, runtime_bridge] = BG.run_search(cmd);
realPathId = extract_real_path(G, BG, pathIdBridge);
cov_set_bridge = calc_coverage(G, realPathId);
cost_bridge = calc_cost(G, realPathId);

if RUN_IRIS_IN_CLUSTERS
    bridgeNodesInPath = BG.graph.Nodes(pathIdBridge + 1, :);
    clusters = unique(bridgeNodesInPath.cluster);
    nClusters = size(clusters, 1);
    for iClust = 1:nClusters
        cIdx = clusters(iClust);
        bridgeNodeInPathInCluster = bridgeNodesInPath(bridgeNodesInPath.cluster == cIdx, :);
        covInBridgeNodesInPath = calc_coverage(BG, bridgeNodeInPathInCluster.id);
        nodesInClusterIdx = find(G.graph.Nodes.cluster == cIdx);
        clusterG = IGraph(G.graph.subgraph(nodesInClusterIdx));
        if length(unique(clusterG.graph.conncomp)) > 1
            % TODO cluster graph is not connected!!
            continue;
        end
        totalCoverageInCluster = calc_coverage(clusterG, clusterG.graph.Nodes.id);
        coverageAchievedInPath = length(covInBridgeNodesInPath) / length(totalCoverageInCluster);
        if coverageAchievedInPath > IRIS_IN_CLUSTER_COV_TH
            continue;
        end
        % Run IRIS in cluster
        % put the bridge node in path first in graph
        bridgeNodeIdx = clusterG.id2idx(bridgeNodeInPathInCluster.id(1));
        newOrder = [bridgeNodeIdx, setdiff(1:clusterG.num_vertices, bridgeNodeIdx)];
        clusterG = IGraph(clusterG.graph.reordernodes(newOrder));
        clusterFile = strrep(bridge_graph_path, '_bridge','_cluster');
        clusterG.write_graph(clusterFile);
        cmd{3} = strrep(cmd{3}, '_bridge', '_cluster');
        cmd{8} = strrep(cmd{8}, '_bridge', '_cluster');
        cmd{9} = num2str(clusterG.num_vertices());
        [clusterPathId, runtime_cluster] = clusterG.run_search(cmd);
        runtime_bridge = runtime_bridge + runtime_cluster;
        pathIrisIds = clusterG.graph.Nodes.id(clusterPathId + 1);
        lastNodeId = clusterG.graph.Nodes.id(clusterPathId(end) + 1);
        lastNodeIdx = clusterG.id2idx(lastNodeId);
        % find shortest path back to bridge node
        pathBackIds = clusterG.graph.Nodes.id(clusterG.graph.shortestpath(lastNodeIdx, 1));
        
        pathInClusterIds = [pathIrisIds(1:end-1)', pathBackIds'];
        
        % add the cluster path to real path
        bridgeIdxInRealPath = find(realPathId == bridgeNodeInPathInCluster.id(1));
        realPathId = [realPathId(1:bridgeIdxInRealPath-1), ...
            pathInClusterIds, ...
            realPathId(bridgeIdxInRealPath+1:end)];
    end
    cost_bridge = calc_cost(G, realPathId);
    cov_set_bridge = calc_coverage(G, realPathId);
end

%% Show path
if contains(env_name, 'syn') || contains(env_name, 'drone')
    if contains(env_name, 'syn')
        PlotEnvironment(params, G, inspectionPoints, obstacles, 'Paths found by IRIS');
        scatter(BG.graph.Nodes.x1, BG.graph.Nodes.x2, 'om', 'Linewidth', 1);
    elseif contains(env_name, 'drone')
        PlotBridgeEnvironment(params, G, inspectionPoints, 'Paths found by IRIS');
        plot3(BG.graph.Nodes.x1, BG.graph.Nodes.x2, BG.graph.Nodes.x3, 'om', 'Linewidth', 1);
    end
    
    % show the original path
    h = show_path(G, pathId, 'go-');
    h.DisplayName = 'Original Path';
    addToLegend = h;
    
    % show the bridge path 
    h = show_path(G, realPathId, 'rx-.');
    h.DisplayName = 'Path Using Bridge Graph';
    addToLegend = [addToLegend h];
    
    % show coverage
    scatter(inspectionPoints(cov_set, 1), inspectionPoints(cov_set, 2), 'og');
    scatter(inspectionPoints(cov_set_bridge, 1), inspectionPoints(cov_set_bridge, 2), 'xr');
    % add covergae to legend
    addToLegend(1).DisplayName = [addToLegend(1).DisplayName ' (' ...
        num2str(roundn(length(cov_set)/size(inspectionPoints, 1),-3)*100) '% Coverage)'];
    
    addToLegend(2).DisplayName = [addToLegend(2).DisplayName ' (' ...
        num2str(roundn(length(cov_set_bridge)/size(inspectionPoints, 1),-3)*100) '% Coverage)'];
    
    legend(addToLegend, 'Location', 'BestOutside');
end
%% Show runtime & results
disp(['Original search runtime: ' num2str(runtime_original)]);
disp(['Bridge graph build time: ' num2str(build_bridge_time)]);
disp(['Bridge search runtime: ' num2str(runtime_bridge)]);
fprintf('Original Covered: %d points\n', length(cov_set))
fprintf('Bridge Covered: %d points\n', length(cov_set_bridge))
fprintf('Original Cost: %.2f\n', cost_orig)
fprintf('Bridge Cost: %.2f\n', cost_bridge)