function M = Edges2M(edges, nPoints)
if nargin < 2
    nPoints = max(reshape(edges(:,1:2)+1,1,[]));
end
M = zeros(nPoints);
for k = 1:size(edges,1)
    M(edges(k,1)+1, edges(k,2)+1) = edges(k,3);
    M(edges(k,2)+1, edges(k,1)+1) = edges(k,3);
end