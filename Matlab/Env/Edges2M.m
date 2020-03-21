function M = Edges2M(edges, nPoints, reassignIdcs)
if ~exist('reassignIdcs', 'var') || isempty(reassignIdcs)
    reassignIdcs = false;
end
idcs = edges(:,1:2)+1;
uIdcs = unique(idcs(:));
if ~exist('nPoints', 'var') || isempty(nPoints)
    if reassignIdcs
        nPoints = length(unique(idcs(:)));
    else
        nPoints = max(reshape(idcs,1,[]));
    end
end
M = zeros(nPoints);
for k = 1:size(edges,1)
    M(uIdcs==idcs(k,1), uIdcs==idcs(k,2)) = 1/edges(k,3);
    M(uIdcs==idcs(k,2), uIdcs==idcs(k,1)) = 1/edges(k,3);
end