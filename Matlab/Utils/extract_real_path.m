function realPathId = extract_real_path(G, BG, pathIdBridge)
% extracts a real path in the original graph, from a path in the bridge
% graph

nodeIds = BG.graph.Nodes.id(pathIdBridge + 1);
nodeIdx = arrayfun(@(x) find(G.graph.Nodes.id == x), nodeIds); % idx in G
realPathIdx = [];
for i = 1:length(nodeIdx) - 1
    sIdx = nodeIdx(i);
    tIdx = nodeIdx(i + 1);
    sId = nodeIds(i);
    tId = nodeIds(i + 1);
    p = G.graph.shortestpath(sIdx, tIdx);
    if length(p) > 2
        % this is a virtual edge, find shortest path withhin cluster
        assert(G.graph.Nodes.cluster(sIdx) == G.graph.Nodes.cluster(tIdx));
        clusterId = G.graph.Nodes.cluster(sIdx);
        nodesInClusterIdx = find(G.graph.Nodes.cluster == clusterId);
        clusterG = G.graph.subgraph(nodesInClusterIdx);
        scIdx = find(clusterG.Nodes.id == sId);
        tcIdx = find(clusterG.Nodes.id == tId);
        p = clusterG.shortestpath(scIdx, tcIdx);
        pIds = clusterG.Nodes.id(p)';
        pIdx = arrayfun(@(x) find(G.graph.Nodes.id == x), pIds); % idx in G
        realPathIdx = [realPathIdx pIdx(2:end)];
    else
        realPathIdx = [realPathIdx sIdx];
    end
end
realPathIdx = [realPathIdx tIdx];
realPathId = realPathIdx - 1;
