function clusters = SpectralClustering(params, points, M, nClusters)
M(M > 0) = 1;
D = diag(sum(M));
L = D-M;
[V,eigenMat] = eig(L);
eigenValues = diag(eigenMat);
if nargin < 4
    dEigens = diff(eigenValues);
    [~, maxIdx] = max(dEigens(2:round(length(dEigens)/2)));
    nClusters = maxIdx+1;
end
kSmallestEigenvalues = eigenValues(1:nClusters+1);
kSmallestEigenvectors = V(:,1:nClusters+1);
nPoints = size(points, 1);
z = zeros(nPoints, nClusters);
for ii = 1:nPoints
    z(ii,:) = 1./kSmallestEigenvalues(2:end).*kSmallestEigenvectors(ii,2:end)';
end
clusters = kmeans(z, nClusters);