function BG = build_bridge_graph(G)

clusters = unique(G.graph.Nodes.cluster);

nClusters = size(clusters, 1);

% For each cluster, find the bridge vertices
bridgeNodeIds = [];
for iClus = 1:nClusters
    
    clIdx = clusters(iClus);
    
    % get vert in this cluster
    vertInClus = G.graph.Nodes(G.graph.Nodes.cluster == clIdx, :);
    
    for vI = 1:size(vertInClus, 1)
        vertNeighbors = G.graph.Nodes(G.graph.neighbors(vertInClus.id(vI) + 1), :);
        vertNeighborsInDiffClust = vertNeighbors(vertNeighbors.cluster ~= clIdx, :);
        if size(vertNeighborsInDiffClust, 1) > 0
            bridgeNodeIds = [bridgeNodeIds vertInClus.id(vI)];
        end
    end
end

% Build the bridge graph, with all edges of bridges (between and within
% clusters)
BG = G.graph.subgraph(bridgeNodeIds + 1);

% Add virtual edges between bridge vert. within clusters
for iClus = 1:nClusters
    clIdx = clusters(iClus);    
    % get vert in this cluster
    vertIdxInClus = find(BG.Nodes.cluster == clIdx);
    dist = BG.distances(vertIdxInClus, vertIdxInClus);
    for i = 1:length(vertIdxInClus)
        for j = i+1:length(vertIdxInClus)
            ee = BG.findedge(vertIdxInClus(i), vertIdxInClus(j));
            if ee == 0
                edgeTable = table([vertIdxInClus(i), vertIdxInClus(j)], 1, 1, 0, 0,...
                    dist(i,j), 1, ...
                    'VariableName', BG.Edges.Properties.VariableNames);
                BG = BG.addedge(edgeTable);
            end
        end
    end
end