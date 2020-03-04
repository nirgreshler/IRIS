function [clustersKmeans, clustersSpectral] = ClusterPoints(points, M, k)
rng(1);
M(M > 0) = 1;
% kmean
D = diag(sum(M));
L = D-M;
[V,eigenMat] = eig(L);

if nargin == 2
    eigenValues = diag(eigenMat);
    dEigens = diff(eigenValues);
    [~, maxIdx] = max(dEigens(2:round(length(dEigens)/2)));
    nClusters = maxIdx+1;
else
    nClusters = k;
end

clustersKmeans = kmeans(points, nClusters);
eigenvalues = diag(D);
kSmallestEigenvalues = eigenvalues(1:nClusters+1);
kSmallestEigenvectors = V(:,1:nClusters+1);

nPoints = size(points, 1);
z = zeros(nPoints, nClusters);
for ii = 1:nPoints
    z(ii,:) = 1./kSmallestEigenvalues(2:end).*kSmallestEigenvectors(ii,2:end)';
end
% clustersSpectral = kmeans(kSmallestEigenvectors, nClusters);
clustersSpectral = kmeans(z, nClusters);