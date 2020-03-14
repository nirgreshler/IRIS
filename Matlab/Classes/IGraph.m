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
                    cluster = table(cluster);
                end
                
                EndNodes = [edge.source+1, edge.target+1];
                EndNodes = table(EndNodes);
                virtual = zeros(size(EndNodes, 1), 1);
                virutal = table(virtual);
                g.graph = graph([EndNodes, edge(:, 3:end), virutal], [vertex, conf(:, 2:end), cluster]);
            elseif isa(filepath, 'graph')
                g.graph = filepath;
            end
        end
        
        function nv = num_vertices(g)
            nv = size(g.graph.Nodes, 1);
        end
        
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
                    if size(vertNeighborsInDiffClust, 1) > 0 || vertInClus.id(vI) == 0 % add the first vertex
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
            % Reorder nodes so that id=0 is first
            [~, order] = sort(BG.Nodes.id);
            BG = BG.reordernodes(order);
            BG = IGraph(BG);
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
                fprintf(fId, strrep(strrep(num2str(pointsInSight(~isnan(pointsInSight))), '   ', ' '), '  ', ' '));
                fprintf(fId, '\n');
            end
            fclose(fId);
            
            % edges
            fId = fopen([pathToWrite '_edge'], 'w');
            for k = 1:size(G.Edges, 1)
                edgeLine = table2array(G.Edges(k, 1:end-1));
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