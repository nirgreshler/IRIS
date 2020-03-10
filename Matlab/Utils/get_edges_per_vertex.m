function edges = get_edges_per_vertex(edges, vertIdx)

firstNode = arrayfun(@(x) any(x == vertIdx), edges(:, 1));
secondNode = arrayfun(@(x) any(x == vertIdx), edges(:, 2));

edges = edges(firstNode & secondNode, :);