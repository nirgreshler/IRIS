function [bridge_conf, bridge_vertex, bridge_edges] = get_bridge_graph(conf, vertex, edges, clusters)

numClusters = length(unique(clusters));

bridgeVert = [];
bridgeEdge = [];

for iClus = 1:numClusters
    vertInClus = vertex(clusters == iClus, 1);
    
    for vI = 1:length(vertInClus)
        vertEdges = [find(edges(:, 1) == vertInClus(vI)); find(edges(:, 2) == vertInClus(vI))];
        vertNeighbors = [edges(edges(:, 1) == vertInClus(vI), 2); edges(edges(:, 2) == vertInClus(vI), 1)];
        neighborsInDifferentClustIdx = clusters(vertNeighbors + 1) ~= iClus;
        if sum(neighborsInDifferentClustIdx)
            % neighborsInDifferentClust = vertNeighbors(neighborsInDifferentClustIdx);
            bridgeVert = [bridgeVert; vertInClus(vI)];
            bridgeEdge = [bridgeEdge; vertEdges(neighborsInDifferentClustIdx)];
        end
    end
    %     bridge_vert{iClus} = bridgeVert;
    
%     scatter(conf(bridgeVert+1, 2), conf(bridgeVert+1, 3), 'x', ...
%         'LineWidth', 1.5, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
end

bridgeVert = unique(bridgeVert);

bridge_vertex = vertex(bridgeVert + 1, :);
bridge_conf = conf(bridgeVert + 1, :);
bridge_edges = edges(bridgeEdge, :);