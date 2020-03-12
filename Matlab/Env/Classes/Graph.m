classdef Graph < handle
    
    properties
        v Vertex
        e Edge
        M
        
        name
    end
    
    methods
        function g = Graph(filepath)
             [conf, vertex, edge] = g.read_graph(filepath);
             
             nV = size(conf, 1);
             nE = size(edge, 1);
             
             g.v = Vertex.empty(nV, 0);
             g.e = Edge.empty(nE, 0);

             g.v = arrayfun(@(i) Vertex(conf(i, :), vertex(i, :)), 1:size(conf, 1))';
             g.e = arrayfun(@(i) Edge(g, edge(i, :)), 1:size(edge, 1))';
             g.edges2M();

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
            opts.DataLines = [1, Inf];
            opts.Delimiter = " ";
            opts.ExtraColumnsRule = "ignore";
            opts.EmptyLineRule = "read";
            opts.ConsecutiveDelimitersRule = "join";
            opts.LeadingDelimitersRule = "ignore";
            % Import the data
            conf = readtable([base_name '_conf'], opts);
            % Convert to output type
            conf = str2double(table2array(conf));
            
            % Clear temporary variables
            clear opts
            
            % Import vertex file
            
            fid = fopen([base_name '_vertex']);
            C = textscan(fid, '%s', 'delimiter','\n');
            fclose(fid);
            C = C{1};
            max_len = max(cellfun(@(x) length(strsplit(x, ' ')), C)) - 1;
            
            opts = delimitedTextImportOptions("NumVariables", max_len);
            
            % Specify range and delimiter
            opts.DataLines = [1, Inf];
            opts.Delimiter = " ";
            
            % Specify column names and types
            opts.ExtraColumnsRule = "ignore";
            opts.EmptyLineRule = "read";
            opts.LeadingDelimitersRule = "ignore";
            
            % Import the data
            vertex = readtable([base_name '_vertex'], opts);
            
            % Convert to output type
            vertex = str2double(table2array(vertex));
            
            % Clear temporary variables
            clear opts
            
            % Import edge data
            opts = delimitedTextImportOptions("NumVariables", 7);
            
            % Specify range and delimiter
            opts.DataLines = [1, Inf];
            opts.Delimiter = " ";
            
            % Specify column names and types
            opts.VariableNames = ["VarName1", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7"];
            opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];
            opts.ExtraColumnsRule = "ignore";
            opts.EmptyLineRule = "read";
            opts.ConsecutiveDelimitersRule = "join";
            opts.LeadingDelimitersRule = "ignore";
            edge = readtable([base_name '_edge'], opts);
            
            % Convert to output type
            edge = table2array(edge);
            
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
        
        function edges2M(g)
            edges = g.e;
            nPoints = g.num_vertices();

            g.M = zeros(nPoints);
            for k = 1:size(edges,1)
                g.M(edges(k).source.id+1, edges(k).target.id+1) = edges(k).cost;
                g.M(edges(k).target.id+1, edges(k).source.id+1) = edges(k).cost;
            end
            assert(issymmetric(g.M));
        end
    end
end