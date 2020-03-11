function [clusters, C] = SpectralClustering(params, points, M, nClusters)
rng(1);
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
if ~isfield(params, 'laplacianType')
    params.laplacianType = 'normal';
end
if params.normalizeM
    M(M > 0) = 1;
end
D = diag(sum(M));
switch params.laplacianType
    case 'normal'
        L = D-M;
    case 'sym'
        L = eye(size(M))-D^(-1/2)*M*D^(-1/2);
    case 'rw'
        L = eye(size(M))-D^-1*M;
end
[V,eigenMat] = eig(L);
eigenValues = diag(eigenMat);
firstValidEig = find(abs(eigenValues) > 1e-12, 1);
if nargin < 4
    dEigens = diff(eigenValues);
    maxVal = max(dEigens(max(params.minClusters-1,1):min(params.maxClusters, length(dEigens))));
    maxIdx = find(dEigens == maxVal);
    nClusters = maxIdx+1;
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
C = C(largeClusters,:);