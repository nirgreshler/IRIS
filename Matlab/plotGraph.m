function plotGraph(varargin)

switch nargin
    case 1
        filename = varargin{1};
        [~, graphName] = fileparts(filename);
        [conf, vertex, edges] = read_graph(filename);
    case 3
        graphName = inputname(1);
        conf = varargin{1};
        vertex = varargin{2};
        edges = varargin{3};
    otherwise
        error('Invalid arguments');
end

s = edges(:, 1) + 1;
t = edges(:, 2) + 1;
weights = edges(:, 7);
G = graph(s, t, weights, strsplit(num2str(0:size(conf, 1)-1)));
figure;
% h = plot(G, 'EdgeLabel',roundn(G.Edges.Weight, -2));
bins = conncomp(G);
numComp = length(unique(bins));
h = plot(G);

if numComp == 1
    title('Graph is fully-connected');
else
    title('Graph NOT is fully-connected');
end

legend(graphName);

% Dijkstra
d = distances(G, vertex(:, 1)+1, vertex(:, 1)+1, 'Method' ,'positive');