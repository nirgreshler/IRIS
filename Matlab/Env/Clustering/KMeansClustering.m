function clusters = KMeansClustering(params, points, arg)
% arg is nClusters or M
if isscalar(arg)
    nClusters = arg;
else
    D = diag(sum(M));
    L = D-M;
    [~,eigenMat] = eig(L);
    eigenValues = diag(eigenMat);
    dEigens = diff(eigenValues);
    [~, maxIdx] = max(dEigens(2:round(length(dEigens)/2)));
    nClusters = maxIdx+1;
end
clusters = kmeans(points, nClusters);