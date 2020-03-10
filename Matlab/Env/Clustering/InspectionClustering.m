function clusters = InspectionClustering(params, points, pointsInSight, M)
if isempty(params)
    params = struct;
end
if ~isfield(params, 'unifyBlindPoints')
    params.unifyBlindPoints = true;
end
if ~isfield(params, 'maxClusters')
    params.maxClusters = 50;
end
nPoints = size(points, 1);
nInspectionPoints = size(pointsInSight,2);
% Remove points with no inspection from clustering
invalidPointsIdcs = find(sum(pointsInSight,2) < nInspectionPoints*0.01);
validPointsIdcs = setxor(1:nPoints, invalidPointsIdcs);
nPoints = length(validPointsIdcs);
pointsInSight = pointsInSight(validPointsIdcs,:);

% Build similirity matrix
Ms = zeros(nPoints);
for k = 1:nPoints-1
    clc
    fprintf('Calculating jacard distance... %.1f%%\n', 100*(k/(nPoints-1)))
    for j = k+1:nPoints
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
validPoints = points(validPointsIdcs,:);
clusters = SpectralClustering(params, validPoints, Ms);
clustersAll(validPointsIdcs) = clusters;
nClusters = length(unique(clusters));

if params.unifyBlindPoints
    clustersAll(invalidPointsIdcs) = nClusters+1;
else
    % Cluster the invalid points
    Mi = M(invalidPointsIdcs, invalidPointsIdcs);
    invalidPoints = points(invalidPointsIdcs,:);
    iClusters = SpectralClustering(params, invalidPoints, Mi);
    iClusters = iClusters+max(unique(clusters));
    clustersAll(invalidPointsIdcs) = iClusters;
end

clusters = clustersAll;