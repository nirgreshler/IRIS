function clusters = InspectionClustering(params, points, pointsInSight)
nPoints = size(points, 1);
% Remove points with no inspection from clustering
invalidPointsIdcs = find(sum(pointsInSight,2) < size(inspectionPoints,1)*0.01);
validPointsIdcs = setxor(1:nPoints, invalidPointsIdcs);
nPoints = length(validPointsIdcs);
pointsInSight = pointsInSight(validPointsIdcs,:);

% Build similirity matrix
Ms = zeros(nPoints);
for k = 1:nPoints-1
    clc
    fprintf('Calculating jacard distance... %.1f%%\n', 100*(k/(nPoints-1)))
    for j = k+1:nPoints
        i1 = find(pointsInSight(k,:));
        i2 = find(pointsInSight(j,:));
        un = sum(pointsInSight(k,:) | pointsInSight(j,:));
        if un == 0
            Ms(k,j) = 0;
            Ms(j,k) = 0;
        else
            in = sum(pointsInSight(k,:) & pointsInSight(j,:));
            Ms(k,j) = in/un;
            Ms(j,k) = Ms(k,j);
        end
    end
end
D = diag(sum(Ms));
L = D-Ms;
[V,eigenMat] = eig(L);

eigenValues = diag(eigenMat);
eigenValues = eigenValues(abs(eigenValues) > 1e-6);
dEigens = diff(eigenValues);
[~, maxIdx] = max(dEigens);
nClusters = maxIdx;

kSmallestEigenvalues = eigenValues(1:nClusters+1);
kSmallestEigenvectors = V(:,1:nClusters+1);

z = zeros(nPoints, nClusters);
for ii = 1:nPoints
    z(ii,:) = 1./kSmallestEigenvalues(2:end).*kSmallestEigenvectors(ii,2:end)';
end
clusters = kmeans(z, nClusters);

clustersAll(validPointsIdcs) = clusters;
clustersAll(invalidPointsIdcs) = nClusters+1;

clusters = clustersAll;