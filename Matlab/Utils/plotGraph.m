function plotGraph(varargin)

switch nargin
    case 1
        if ischar(varargin{1})
            filename = varargin{1};
            [~, graphName] = fileparts(filename);
            [conf, vertex, edges] = read_graph(filename);
        elseif isa(varargin{1}, 'graph')
            G = varargin{1};
            graphName = inputname(1);
        end
    case 3
        graphName = inputname(1);
        conf = varargin{1};
        vertex = varargin{2};
        edges = varargin{3};
    otherwise
        error('Invalid arguments');
end

if ~exist('G', 'var')
    G = get_graph(conf, vertex, edges);
end
figure;
% h = plot(G, 'EdgeLabel',roundn(G.Edges.Weight, -2));
bins = conncomp(G);
numComp = length(unique(bins));
plot(G);

if numComp == 1
    title('Graph is fully-connected');
else
    title('Graph NOT is fully-connected');
end

legend(graphName);