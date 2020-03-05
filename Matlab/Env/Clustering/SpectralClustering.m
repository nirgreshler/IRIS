function clusters = SpectralClustering(params, points, M, nClusters)
if isempty(params)
    params = struct;
end
if ~isfield(params, 'maxClusters')
    params.maxClusters = 50;
end
if ~isfield(params, 'minClusters')
    params.minClusters = 2;
end
if ~isfield(params, 'normalizeM')
    params.normalizeM = true;
end
if params.normalizeM
    M(M > 0) = 1;
end
D = diag(sum(M));
L = D-M;
[V,eigenMat] = eig(L);
eigenValues = diag(eigenMat);
if nargin < 4
    dEigens = diff(eigenValues);
    [~, maxIdx] = max(dEigens(2:round(length(dEigens)/2)));
    nClusters = max(min(maxIdx+1, params.maxClusters), params.minClusters);
end
kSmallestEigenvalues = eigenValues(1:nClusters+1);
kSmallestEigenvectors = V(:,1:nClusters+1);
nPoints = size(points, 1);
z = zeros(nPoints, nClusters);
for ii = 1:nPoints
    z(ii,:) = 1./kSmallestEigenvalues(2:end).*kSmallestEigenvectors(ii,2:end)';
end
[clusters, C] = kmeans(z, nClusters);

% Remove small clusters
N = histcounts(clusters, nClusters);
smallClusters = find(N < 0.01*nPoints);
largeClusters = setxor(1:nClusters, smallClusters);
for c = 1:length(smallClusters)
    pointsOfSmallClusters = find(clusters == smallClusters(c));
    for k = 1:length(pointsOfSmallClusters)
        [~,idx] = min(sqrt(sum((z(pointsOfSmallClusters(k),:)-C(largeClusters,:)).^2,2)));
        clusters(pointsOfSmallClusters(k)) = largeClusters(idx);
    end
end