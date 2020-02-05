function [clustersKmeans, clustersSpectral] = ClusterPoints(points, M, k)
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
kSmallestEigenvectors = V(:,1:nClusters+1);
clustersSpectral = kmeans(kSmallestEigenvectors, nClusters);