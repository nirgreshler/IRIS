function M = Edges2M(edges)
M = zeros(max(reshape(edges(:,1:2)+1,1,[])));
for k = 1:size(edges,1)
    M(edges(k,1)+1, edges(k,2)+1) = edges(k,3);
    M(edges(k,2)+1, edges(k,1)+1) = edges(k,3);
end