function h = show_path(G, pathIdx, cml)

for i = 1:length(pathIdx) - 1
    h = plot([G.graph.Nodes.x1(pathIdx(i) + 1), G.graph.Nodes.x1(pathIdx(i + 1) + 1)], ...
        [G.graph.Nodes.x2(pathIdx(i) + 1), G.graph.Nodes.x2(pathIdx(i + 1) + 1)], ...
        cml(1), 'Marker', cml(2), 'LineStyle', cml(3:end), 'LineWidth', 1, 'DisplayName', 'Original');
end