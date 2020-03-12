function BG = build_bridge_graph(G, clusters)

nClusters = size(clusters, 1);

% For each cluster, find the bridge vertices
for iClus = 1:nClusters
    vertInClus = clusters(iClus).v;
    
    for vI = 1:length(vertInClus)
        vertEdges = G.getVertEdges(vertInClus(vI));
    end
    
end

% Find all edges that connect all bridge vertices between clusters

% Add edges between bridge vert. within clusters

% Add virtual edges between bridge vert. within clusters