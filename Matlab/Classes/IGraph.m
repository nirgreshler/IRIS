classdef IGraph < handle
    
    properties
        graph
        
        v Vertex
        e Edge
        M
        
        name
    end
    
    methods
        function g = IGraph(filepath, clusParams)
             [conf, vertex, edge] = g.read_graph(filepath);
             
             nV = size(conf, 1);
             nE = size(edge, 1);

             g.v = Vertex.empty(nV, 0);
             g.e = Edge.empty(nE, 0);

             g.v = arrayfun(@(i) Vertex(table2array(conf(i, :)), [table2array(vertex(i, 1:3)), vertex.vis{i}]), 1:size(conf, 1))';
             g.e = arrayfun(@(i) Edge(g, table2array(edge(i, :))), 1:size(edge, 1))';
             g.M = g.edges2M(table2array(edge));
            
             % Perform clustering
             cluster = SpectralClustering(clusParams, table2array(conf(:, 2:end)), g.M);
             cluster = table(cluster);
             
             % g.graph = graph(M, [vertex, conf(:, 2:end)]);
             EndNodes = [edge.source+1, edge.target+1];
             EndNodes = table(EndNodes);
             virtual = zeros(size(EndNodes, 1), 1);
             virutal = table(virtual);
             g.graph = graph([EndNodes, edge(:, 3:end), virutal], [vertex, conf(:, 2:end), cluster]);
%              g.graph = g.graph.addnode([vertex, conf(:, 2:end)]);
%              g.graph = g.graph.addedge([EndNodes, edge(:, 3:end)]);
        end
        
        function nodes = getNodeById(g, id)
            ids = cell2mat({g.v.id});
            nodes = arrayfun(@(x) g.v(ids == x), id);
        end
        
        function nv = num_vertices(g)
            nv = size(g.v, 1);
        end
        
        function e = getVertEdges(g, v_)
            if ~isa(v_, 'Vertex')
                v_ = g.getNodeById(v_);
            end
            
            sources = cellfun(@(v) v.id, {g.e.source});
            targets = cellfun(@(v) v.id, {g.e.target});
            
            [find(sources == v_.id)'; find(targets == v_.id)']
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
        
        function M = edges2M(g, edges)
            nPoints = g.num_vertices();

            M = zeros(nPoints);
            for k = 1:size(edges,1)
                M(edges(k, 1)+1, edges(k, 2)+1) = edges(k, end);
                M(edges(k, 2)+1, edges(k, 1)+1) = edges(k, end);
            end
            assert(issymmetric(M));
        end
    end
end