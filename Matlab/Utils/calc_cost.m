function cov_set = calc_cost(G, pathId)

if isa(G, 'IGraph')
    G = G.graph;
end

cov_set = [];
for i = 1:length(pathId)
    cov = G.Nodes.vis{G.Nodes.id == pathId(i)};
    cov_set = [cov_set cov(~isnan(cov))];
end
cov_set = unique(cov_set) + 1;