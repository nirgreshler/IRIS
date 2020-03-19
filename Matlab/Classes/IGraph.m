classdef IGraph < handle
    
    properties
        graph
        M
        name
    end
    
    methods
        function g = IGraph(filepath, clustering)
            
            if ischar(filepath)
                [~, g.name] = fileparts(filepath);
                [conf, vertex, edge] = g.read_graph(filepath);

                g.M = g.edges2M(size(conf, 1), table2array(edge));
                
                % Perform clustering
                if nargin > 1
                    cluster = clustering.cluster(table2array(conf(:, 2:3)), g.M);
                    % make the clusters serial 1..n
                    uClusters = unique(cluster);
                    for i = 1:length(uClusters)
                        cluster(cluster == uClusters(i)) = i;
                    end
                    cluster = table(cluster);
                end
                
                EndNodes = [edge.source+1, edge.target+1];
                EndNodes = table(EndNodes);
                virtual = zeros(size(EndNodes, 1), 1);
                virutalEdges = table(virtual);
                virtual = zeros(size(conf, 1), 1);
                virutalNodes = table(virtual);
                g.graph = graph([EndNodes, edge(:, 3:end), virutalEdges], [vertex, conf(:, 2:end), cluster, virutalNodes]);
            elseif isa(filepath, 'graph')
                g.graph = filepath;
            end
        end
        
        function nv = num_vertices(g)
            nv = size(g.graph.Nodes, 1);
        end
        
        function idx = id2idx(G, ids)
            idx = arrayfun(@(x) find(G.graph.Nodes.id == x), ids);
        end
        
        function id = idx2id(G, idx)
            id = G.graph.Nodes.id(idx);
        end
        
        function BG = build_bridge_graph(G, addVirtVert, MIN_CONN)
            
            if nargin == 1
                addVirtVert = true;
                MIN_CONN = inf;
            elseif nargin == 2
                MIN_CONN = inf;
            end
            
            clusters = unique(G.graph.Nodes.cluster);
            
            nClusters = size(clusters, 1);
            
            % limit number of bridge nodes
            connected_clusters = zeros(nClusters, nClusters);
            
            % For each cluster, find the bridge vertices
            bridgeNodeIds = [];
            for iClus = 1:nClusters
                
                clIdx = clusters(iClus);
                
                % get vert in this cluster
                vertInClus = G.graph.Nodes(G.graph.Nodes.cluster == clIdx, :);
                
                for vI = 1:size(vertInClus, 1)
                    vertNeighbors = G.graph.Nodes(G.graph.neighbors(G.id2idx(vertInClus.id(vI))), :);
                    vertNeighborsInDiffClust = vertNeighbors(vertNeighbors.cluster ~= clIdx, :);
                    if size(vertNeighborsInDiffClust, 1) > 0 || vertInClus.id(vI) == 0 % add the first vertex
                        % only add bridge nodes that add new connection
                        if any(connected_clusters(clIdx, vertNeighborsInDiffClust.cluster) < MIN_CONN) || vertInClus.id(vI) == 0 % add the first vertex
                            bridgeNodeIds = [bridgeNodeIds vertInClus.id(vI)];
                            connected_clusters(clIdx, vertNeighborsInDiffClust.cluster) = ...
                                connected_clusters(clIdx, vertNeighborsInDiffClust.cluster) + 1;
                        end
                    end
                end
            end
            
            % Build the bridge graph, with all edges of bridges (between and within
            % clusters)
            BG = G.graph.subgraph(G.id2idx(bridgeNodeIds));
            A = BG.adjacency;
            
            % Add virtual edges between bridge vert. within clusters
            ptsPerCluster = histcounts(BG.Nodes.cluster, nClusters);
            maxVirtEdges = 2 * sum((ptsPerCluster .* (ptsPerCluster - 1))/2);
            maxVirtVerts = sum((ptsPerCluster .* (ptsPerCluster - 1))/2);
            vEdgesArray = zeros(maxVirtEdges, 8);
            vVertsArray = cell(maxVirtVerts, 8); % TODO
            vEIdx = 1;
            vVIdx = 1;
            nextVertId = max(G.graph.Nodes.id) + 1; % generate new unique id
            nextVertIdxInBridgeGraph = BG.numnodes() + 1;
            
%             addVirtEdgesIds = cell(G.graph.numnodes(), 1);
%             addedVirtVertIds = [];
            for iClus = 1:nClusters
                clIdx = clusters(iClus);
                % get vert in this cluster
                vertIdxInBridgeClus = find(BG.Nodes.cluster == clIdx);
                vertIdInClus = BG.Nodes.id(vertIdxInBridgeClus);
                
                % Build a graph of cluster
                nodesInClusterIdx = find(G.graph.Nodes.cluster == clIdx);
                clusterG = G.graph.subgraph(nodesInClusterIdx);

                % assert(length(unique(clusterG.conncomp)) == 1);
                vertIdxInClus = arrayfun(@(x) find(clusterG.Nodes.id == x), vertIdInClus);
                
                % dist = clusterG.distances(vertIdxInClus, vertIdxInClus);
                
%                 if length(unique(clusterG.conncomp)) == 1
%                     continue;
%                 end
                
                for i = 1:length(vertIdxInBridgeClus)
                    maxVertToConnect = round(length(vertIdxInBridgeClus) / 5);
                    connectedVert = 0;
                    otherVert = setdiff(vertIdxInBridgeClus, vertIdxInBridgeClus(i));
                    randVertToConnect = otherVert(randperm(length(otherVert)));
                    % randVertToConnect = randsample(setdiff(vertIdxInBridgeClus, vertIdxInBridgeClus(i)), maxVertToConnect);
                    for jI = 1:length(randVertToConnect)
                        j = find(vertIdxInBridgeClus == randVertToConnect(jI));
                        if connectedVert == maxVertToConnect
                            break;
                        end
                        alreadyConnected = any(sum(vEdgesArray(:, 1:2) == [vertIdxInBridgeClus(j), vertIdxInBridgeClus(i)], 2) == 2);
                        if alreadyConnected
                            continue;
                        end
                        % ee = BG.findedge(vertIdxInBridgeClus(i), vertIdxInBridgeClus(j));
                        ee = A(vertIdxInBridgeClus(i), vertIdxInBridgeClus(j));
                        if full(ee) == 0
                            % there is not an edge between the bridge nodes
                            [P, d] = clusterG.shortestpath(vertIdxInClus(i), vertIdxInClus(j));
                            if isempty(P)
                                % if no path exist, ignore it
                                continue;
                            end
                            innerPathIdx = P(2:end-1);
                            assert(~isempty(innerPathIdx));
                            innerPathId = clusterG.Nodes.id(innerPathIdx);
                            tic
                            inspectPtsId = calc_coverage(clusterG, innerPathId) - 1;
                            time_vis = toc;
                            if isempty(inspectPtsId) || ~addVirtVert
                                % inner path does not add coverage, just
                                % add one virtual edge
                                vEdgesArray(vEIdx, :) = [vertIdxInBridgeClus(i), vertIdxInBridgeClus(j), 1, 1, 0, 0,...
                                    d, 1];
                                vEIdx = vEIdx + 1;
                                connectedVert = connectedVert + 1;
                            else   
                                % inner path adds coverage, add one virtual
                                % vertex and two virutal edges
                                vEdgesArray(vEIdx, :) = [vertIdxInBridgeClus(i), nextVertIdxInBridgeGraph, 1, 1, 0, 0,...
                                    d/2, 1];
                                vEIdx = vEIdx + 1;
                                vEdgesArray(vEIdx, :) = [nextVertIdxInBridgeGraph, vertIdxInBridgeClus(j), 1, 1, 0, 0,...
                                    d/2, 1];
                                vEIdx = vEIdx + 1;
                                
                                % Add virtual vertex
                                new_conf = [clusterG.Nodes.x1(innerPathIdx(1)), ...
                                    clusterG.Nodes.x2(innerPathIdx(1))]; % TODO
                                
                                vVertsArray(vVIdx, :) = [nextVertId, 0, time_vis, {inspectPtsId}, new_conf(1), new_conf(2), clIdx, 1];
                                vVIdx = vVIdx + 1;
                                nextVertId = nextVertId + 1;
                                nextVertIdxInBridgeGraph = nextVertIdxInBridgeGraph + 1;
                            end
                        end
                    end
                end
            end
            
            % remove unused rows
            vEdgesArray = vEdgesArray(1:vEIdx-1, :);
            vVertsArray = vVertsArray(1:vVIdx-1, :); 
            
            % create and add tables
            edgeTable = table([vEdgesArray(:, 1), vEdgesArray(:, 2)], ...
                vEdgesArray(:, 3), vEdgesArray(:, 4), vEdgesArray(:, 5), ...
                vEdgesArray(:, 6), vEdgesArray(:, 7), vEdgesArray(:, 8), ...
                'VariableName', BG.Edges.Properties.VariableNames);
            if addVirtVert
                vertTable = table(cell2mat(vVertsArray(:, 1)), cell2mat(vVertsArray(:, 2)), ...
                    cell2mat(vVertsArray(:, 3)), vVertsArray(:, 4), cell2mat(vVertsArray(:, 5)), ...
                    cell2mat(vVertsArray(:, 6)), cell2mat(vVertsArray(:, 7)), cell2mat(vVertsArray(:, 8)), ...
                    'VariableName', BG.Nodes.Properties.VariableNames);
                BG = BG.addnode(vertTable);
            end
            BG = BG.addedge(edgeTable);
            
            % Reorder nodes so that id=0 is first
            [~, order] = sort(BG.Nodes.id);
            BG = BG.reordernodes(order);
            BG = IGraph(BG);
        end
        
        function [pathId, runtime] = run_search(G, cmd)
            tic
            status = system(strjoin(cmd));
            runtime = toc;
            if status
                error('Failed to run');
            end
            
            res_file = [cmd{3} '_result'];
            sp = strsplit(res_file, '/');
            sp = sp(3:end);
            sp{1} = [sp{1} ':'];
            res_file = strjoin(sp, '\');
            pathId = read_result(res_file);
        end
        
        function write_graph(G, pathToWrite)
            assert(isa(G, 'IGraph'));
            G = G.graph;
            nVars = length(G.Nodes.Properties.VariableNames);
            confVarIdx = [];
            for i = 1:nVars
                for j = 1:nVars
                    if strcmp(G.Nodes.Properties.VariableNames{j}, ['x' num2str(i)])
                        confVarIdx = [confVarIdx j];
                    end
                end
            end
            conf = table2array(G.Nodes(:, confVarIdx));
            
            nPoints = size(conf, 1);
            
            % conf
            fId = fopen([pathToWrite '_conf'], 'w');
            for k = 1:nPoints
                fprintf(fId, '%d %f %f\n', k-1, conf(k, 1), conf(k, 2));
            end
            fclose(fId);
            
            % vertex
            fId = fopen([pathToWrite '_vertex'], 'w');
            for k = 1:nPoints
                fprintf(fId, '%d %f %f ', k-1, G.Nodes.time_vis(k), G.Nodes.time_build(k));
                pointsInSight = G.Nodes.vis{k};
                pointsInSight = pointsInSight(~isnan(pointsInSight));
                fprintf(fId, strjoin(arrayfun(@(x) num2str(x), pointsInSight, 'UniformOutput', false), ' '));
                fprintf(fId, '\n');
            end
            fclose(fId);
            
            % edges
            fId = fopen([pathToWrite '_edge'], 'w');
            edgeArray = table2array(G.Edges(:, 1:end-1));
            for k = 1:size(G.Edges, 1)
                edgeLine = edgeArray(k, :);
                % fix indices
                edgeLine(1) = edgeLine(1) - 1;
                edgeLine(2) = edgeLine(2) - 1;
                fprintf(fId, '%d %d %d %d %d %d %f\n', edgeLine);
            end
            fclose(fId);
        end
    end
    
    methods(Access=private)
        function [conf, vertex, edge] = read_graph(g, base_name)
            conf_dims = g.get_dims(base_name);
            
            % Import conf file
            opts = delimitedTextImportOptions("NumVariables", conf_dims + 1);
            opts.VariableNames{1} = 'id';
            opts.VariableTypes{1} = 'double';
            for i = 2:length(opts.VariableNames)
                opts.VariableNames{i} = ['x' num2str(i - 1)];
                opts.VariableTypes{i} = 'double';
            end
            opts.DataLines = [1, Inf];
            opts.Delimiter = " ";
            opts.ExtraColumnsRule = "ignore";
            opts.EmptyLineRule = "read";
            opts.ConsecutiveDelimitersRule = "join";
            opts.LeadingDelimitersRule = "ignore";
            % Import the data
            conf = readtable([base_name '_conf'], opts);
            % Convert to output type
            %             conf = str2double(table2array(conf));
            
            % Clear temporary variables
            clear opts
            
            % Import vertex file
            
            fid = fopen([base_name '_vertex']);
            C = textscan(fid, '%s', 'delimiter','\n');
            fclose(fid);
            C = C{1};
            
            id = zeros(size(C, 1), 1);
            time_vis = zeros(size(C, 1), 1);
            time_build = zeros(size(C, 1), 1);
            vis = cell(size(C, 1), 1);
            
            for i = 1:size(C, 1)
                lSp = strsplit(C{i});
                id(i) = str2double(lSp{1});
                time_vis(i) = str2double(lSp{2});
                time_build(i) = str2double(lSp{3});
                vis{i} = cellfun(@(x) str2double(x), lSp(4:end));
            end
            
            vertex = table(id, time_vis, time_build, vis);
            
            %             max_len = max(cellfun(@(x) length(strsplit(x, ' ')), C)) - 1;
            %
            %             opts = delimitedTextImportOptions("NumVariables", max_len);
            %
            %             % Specify range and delimiter
            %             opts.DataLines = [1, Inf];
            %             opts.Delimiter = " ";
            %
            %             % Specify column names and types
            %             opts.ExtraColumnsRule = "ignore";
            %             opts.EmptyLineRule = "read";
            %             opts.LeadingDelimitersRule = "ignore";
            %
            %             % Import the data
            %             vertex = readtable([base_name '_vertex'], opts);
            %
            %             % Convert to output type
            %             vertex = str2double(table2array(vertex));
            %
            %             % Clear temporary variables
            %             clear opts
            
            % Import edge data
            opts = delimitedTextImportOptions("NumVariables", 7);
            
            % Specify range and delimiter
            opts.DataLines = [1, Inf];
            opts.Delimiter = " ";
            
            % Specify column names and types
            opts.VariableNames = ["source", "target", "checked", "valid", "time_forward_kinematics", "time_collision_detection", "Weight"];
            opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];
            opts.ExtraColumnsRule = "ignore";
            opts.EmptyLineRule = "read";
            opts.ConsecutiveDelimitersRule = "join";
            opts.LeadingDelimitersRule = "ignore";
            edge = readtable([base_name '_edge'], opts);
            
            % Convert to output type
            %             edge = table2array(edge);
            
            % Clear temporary variables
            clear opts
        end
        
        function conf_dims = get_dims(g, name)
            if contains(name, 'drone')
                conf_dims = 5;
            elseif contains(name, 'planar')
                conf_dims = 5;
            elseif contains(name, 'crisp')
                conf_dims = 45;
            elseif contains(name, 'syn')
                conf_dims = 2;
            else
                conf_dims = 5;
            end
        end
        
        function M = edges2M(~, nPoints, edges)
            M = zeros(nPoints);
            for k = 1:size(edges,1)
                M(edges(k, 1)+1, edges(k, 2)+1) = edges(k, end);
                M(edges(k, 2)+1, edges(k, 1)+1) = edges(k, end);
            end
            assert(issymmetric(M));
        end
    end
end