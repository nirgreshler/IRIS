clear
close all
clc
roomSize = 10;
envGrid = 0.01;
pointsGrid = 0.1;
doorSize = 0.5;
nPoints = 200;
connectionRadius = 2;
samplingMethod = 'Uniform'; % 'Uniform' / 'Sobol'
inspectionPoints = GetInspectionPoints(roomSize, pointsGrid);
[obstacles, doors] = GetObstacles(roomSize, doorSize, envGrid);
skip = randi(100000);
switch samplingMethod
    case 'Uniform'
        points = envGrid*round(rand(nPoints,2)*roomSize/envGrid);
    case 'Sobol'
        sampler = sobolset(2);
        sampler.Skip = skip;
        points = sampler.net(nPoints)*roomSize;
end
% Remove invalid points
points = unique(points, 'rows');
validIdcs = points(:,1) ~= roomSize/2 & points(:,1) ~= 0 & points(:,1) ~= roomSize &...
    points(:,2) ~= roomSize/2 & points(:,2) ~= 0 & points(:,2) ~= roomSize;
points = points(validIdcs,:);
nPoints = size(points,1);

% Build adjacency matrix
M = zeros(nPoints); 
for k = 1:nPoints
    p1 = points(k,:);
    for m = k+1:nPoints
        p2 = points(m,:);
        if norm(p1-p2) <= connectionRadius && CollisionDetector(p1, p2, doors, envGrid, doorSize)
            M(k,m) = 1;
            M(m,k) = 1;
        end
    end
end
%% Clustering
% kmean
D = diag(sum(M));
L = D-M;
[V,D] = eig(L);
eigenValues = diag(D);
[~, maxIdx] = max(diff(eigenValues));
nClusters = maxIdx;
clusters = kmeans(points, nClusters);
kSmallestEigenvalues = eigenValues(1:nClusters+1);
kSmallestEigenvectors = V(:,1:nClusters+1);
clustersSpctral = kmeans(kSmallestEigenvectors, nClusters);

%% G inspection point for each point
pointsInSight = zeros(nPoints, size(inspectionPoints,1));
for k = 1:nPoints
    for p = 1:size(inspectionPoints,1)
        if CollisionDetector(points(k,:), inspectionPoints(p,:), doors, envGrid, doorSize)
            pointsInSight(k,p) = 1;
        end
    end
end
%% Plot enviroment
PlotEnvironment(points, clusters, M, inspectionPoints, obstacles, roomSize, 'Clustered with K-Means');
PlotEnvironment(points, clustersSpctral, M, inspectionPoints, obstacles, roomSize, 'Clustered with Spectral Clustering');
%% Write text files
fId = fopen('syn_conf', 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f\n', k-1, points(k,1), points(k,2));
end
fclose(fId);

fId = fopen('syn_vertex', 'w');
for k = 1:nPoints
    fprintf(fId, '%d 0 0 ', k-1);
    fprintf(fId, strrep(strrep(num2str(find(pointsInSight(k,:))), '   ', ' '), '  ', ' '));
    fprintf(fId, '\n');
end
fclose(fId);

fId = fopen('syn_edge', 'w');
for k = 1:nPoints
    for p = k+1:nPoints
        if M(k,p) == 1
            fprintf(fId, '%d %d 1 1 0 0 %f\n', k-1, p-1, norm(points(k,:)-points(p,:)));
        end
    end
end
fclose(fId);