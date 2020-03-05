function [G, d] = get_graph(conf, vertex, edges)

% s = edges(:, 1) + 1;
% t = edges(:, 2) + 1;

s = strsplit(num2str(edges(:, 1)'));
t = strsplit(num2str(edges(:, 2)'));
weights = edges(:, 7);

G = graph(s, t, weights);

% Dijkstra
d = distances(G, 'Method' ,'positive');