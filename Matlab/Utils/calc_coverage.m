function cov_set = calc_coverage(G, pathId)

cov_set = [];
for i = 1:length(pathId)
    cov = G.graph.Nodes.vis{G.graph.Nodes.id == pathId(i)};
    cov_set = [cov_set cov(~isnan(cov))];
end
cov_set = unique(cov_set) + 1;