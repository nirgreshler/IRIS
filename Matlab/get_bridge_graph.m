function [bridge_conf, bridge_vertex, bridge_edges, vert_idx_mapping, vedges_mapping] = get_bridge_graph(conf, vertex, edges, clusters, obstacles, params, addOnlyVirtualEdges)

if nargin == 6
    addOnlyVirtualEdges = false;
end

numClusters = length(unique(clusters));

bridgeVert = [];
bridgeEdge = [];
vedges_mapping = cell(0, 2); % maps a virtual edge to a list of vertex of the real path

for iClus = 1:numClusters
    vertInClus = vertex(clusters == iClus, 1);
    
    for vI = 1:length(vertInClus)
        vertEdges = [find(edges(:, 1) == vertInClus(vI)); find(edges(:, 2) == vertInClus(vI))];
        vertNeighbors = [edges(edges(:, 1) == vertInClus(vI), 2); edges(edges(:, 2) == vertInClus(vI), 1)];
        neighborsInDifferentClustIdx = clusters(vertNeighbors + 1) ~= iClus;
        if sum(neighborsInDifferentClustIdx) || vertInClus(vI) == 0 % always keep the first vertex
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

% add the inner edges within a cluster
virtual_edges_added = 0;
virtual_nodes_added = 0;
numClusters = length(unique(clusters));
newVertexIdx = size(vertex, 1);
for iClus = 1:numClusters
    vertInClus = vertex(clusters == iClus, 1); % All vertices ID in this cluster
    vertInClusNames = strsplit(num2str(vertInClus'));
    vertInClusInBridge = intersect(vertInClus, bridge_vertex(:, 1)); % All bridge-vertices ID in this cluster
    vertInClusInBridgeNames = strsplit(num2str(vertInClusInBridge'));
    
    % We want to find the shortest path between all bridge nodes to all
    % bridge nodes
    points = conf(vertInClus + 1, 2:3);
    clustEdges = get_edges_per_vertex(edges, vertInClus);
    Mc = Edges2M(clustEdges(:,[1 2 7]), [], true);
%     Mc = BuildAdjcancyMatrix(points, obstacles, params.connectionRadius); % TODO take from original M
    %     g = graph(Mc, vertInClusNames);
    %     d = distances(g, vertInClusInBridgeNames, vertInClusInBridgeNames, 'Method' ,'positive');
    %     [TR, d] = shortestpathtree(g ,vertInClusInBridgeNames, vertInClusInBridgeNames, 'Method' ,'positive');
    %     d = roundn(d, -4);
    %     assert(issymmetric(d));
    
    for i = 1:length(vertInClusInBridge)-1
        for j = i+1:length(vertInClusInBridge)
            startIdx = find(all(points == conf(vertInClusInBridge(i) + 1, 2:3),2));
            goalIdx = find(all(points == conf(vertInClusInBridge(j) + 1, 2:3),2));
            [pathFound, cost] = AStar(points, startIdx, goalIdx, Mc);
            if ~isempty(pathFound)
                % get the coverage of all points in the path
                innerPath = pathFound(2:end-1, :);
                innerPathIdcsInOriginalGraph = vertInClus(innerPath(:,1));
                
                if isempty(innerPath)
                    % the two bridge nodes are already connected in the
                    % original graph, add it to the bridge graph
                    edgeIdx = find(edges(:, 1) == vertInClusInBridge(i) & edges(:, 2) == vertInClusInBridge(j));
                    assert(~isempty(edgeIdx));
                    bridge_edges = [bridge_edges; edges(edgeIdx, :)];
                else
                    % get the coverage of inner path
                    cov = unique(vertex(innerPath(:, 1) + 1, 4:end));
                    innerPathCov = cov(~isnan(cov));
                    innerPathCov = reshape(innerPathCov, 1, length(innerPathCov));
                    
                    if isempty(innerPathCov) || addOnlyVirtualEdges
                        % inner path has no coverage, just add virtual edge
                        newVirtualEdge = [vertInClusInBridge(i), vertInClusInBridge(j), 1, 1, 0, 0, cost]; % TODO 1 1 0 0
                        virtual_edges_added = virtual_edges_added + 2;
                        
                        % Add the virtual edge to the bridge graph
                        bridge_edges = [bridge_edges; newVirtualEdge];
                        vedges_mapping{size(vedges_mapping,1) + 1, 1} = size(bridge_edges, 1);
                        vedges_mapping{size(vedges_mapping,1), 2} = innerPathIdcsInOriginalGraph;
                    else
                        % Add a virutal vertex
                        newVertexIdx = newVertexIdx + 1;
                        newVirutalVertex = [newVertexIdx, 0, 0, innerPathCov];
                        newVirutalConf = [newVertexIdx, innerPath(1, 2:3)]; % the conf. of first node in inner path
                        
                        padLen = size(bridge_vertex, 2) - length(newVirutalVertex);
                        
                        if padLen > 0
                            newVirutalVertex = [newVirutalVertex, repmat(nan, 1, padLen)];
                        else
                            padLen = -padLen;
                            bridge_vertex = [bridge_vertex, repmat(nan, size(bridge_vertex, 1), padLen)];
                        end
                        
                        % Add two virtual edges
                        newVirtualEdge1 = [vertInClusInBridge(i), newVertexIdx, 1, 1, 0, 0, cost/2]; % TODO 1 1 0 0
                        newVirtualEdge2 = [newVertexIdx, vertInClusInBridge(j), 1, 1, 0, 0, cost/2]; % TODO 1 1 0 0
                        
                        virtual_edges_added = virtual_edges_added + 2;
                        
                        % Add the virtual nodes and edges to the bridge graph
                        bridge_edges = [bridge_edges; newVirtualEdge1; newVirtualEdge2];
                        bridge_vertex = [bridge_vertex; newVirutalVertex];
                        bridge_conf = [bridge_conf; newVirutalConf];
                    end
                end
            end
        end
    end
end
disp(['Added ' num2str(virtual_edges_added) ' edges within clusters']);

[bridge_edges, uIdx] = unique(bridge_edges, 'rows', 'stable');
for ii = 1:size(vedges_mapping, 1)
    newIdx = find(vedges_mapping{ii, 1} == uIdx);
    vedges_mapping{ii, 1} = newIdx;
end

% need to fix indices
nNodes = size(bridge_vertex(:, 1), 1);
vert_idx_mapping = bridge_vertex(:, 1);
bridge_vertex(:, 1) = 0:nNodes-1;
bridge_conf(:, 1) = 0:nNodes-1;
edgesIdx = bridge_edges(:, 1:2);
for i = 0:nNodes-1
    edgesIdx(edgesIdx == vert_idx_mapping(i+1)) = i;
end
bridge_edges(:, 1:2) = edgesIdx;