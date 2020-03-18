function cost = calc_cost(G, pathId)

if isa(G, 'IGraph')
    G = G.graph;
end

cost = 0;
for i = 1:length(pathId)-1
    edgeIdx = find((G.Edges.EndNodes(:,1)-1 == pathId(i) & G.Edges.EndNodes(:,2)-1 == pathId(i+1)) |...
    (G.Edges.EndNodes(:,1)-1 == pathId(i+1) & G.Edges.EndNodes(:,2)-1 == pathId(i)));
    cost = cost+G.Edges(edgeIdx,:).Weight;
end