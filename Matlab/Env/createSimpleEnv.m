clear
close all
clc
nSqrRooms = 3;
homeSize = 9;%nSqrRooms^2;
nRooms = nSqrRooms^2;
roomSize = homeSize/nSqrRooms;
envGrid = 0.01;
doorSize = 0.25;
nPoints = 2000;
nInspectionPoints = 400;
connectionRadius = 0.8;
sightRadius = roomSize*sqrt(2)/2;
samplingMethod = 'RRT'; % 'Uniform' / 'Sobol' / 'RRT'
fol = fileparts(mfilename('fullpath'));
rng('shuffle')
outputFolder = fullfile(fol, '..', 'Graphs', filesep);
saveEnv = true;
params.homeSize = homeSize;
params.connectionRadius = connectionRadius;
params.sightRadius = sightRadius;
params.doorSize = doorSize;

%% Create points
inspectionPoints = GetInspectionPoints(homeSize, nInspectionPoints);
[obstacles, doors] = GetObstacles(homeSize, nRooms, doorSize, envGrid);
skip = randi(100000);
switch samplingMethod
    case 'Uniform'
        points = envGrid*round(rand(nPoints,2)*homeSize/envGrid);
    case 'Sobol'
        sampler = sobolset(2);
        sampler.Skip = skip;
        points = sampler.net(nPoints)*homeSize;
    case 'RRT'
        eta = 0.5;
        startPoint = [envGrid envGrid];
        points = startPoint;
        while size(points,1) < nPoints
            m = 0;
            while m == 0
                randPoint = envGrid*round(rand(1,2)*homeSize/envGrid);
                [m,i] = min(sqrt(sum((points-randPoint).^2,2)));
            end
            direction = randPoint-points(i,:);
            direction = direction./norm(direction);
            newPoint = points(i,:)+direction*min(eta, norm(points(i,:)-randPoint));
            if CollisionDetector(points(i,:), newPoint, obstacles, connectionRadius)
                points = [points; newPoint];
            end
        end
        nPoints = size(points,1);
end
%% Remove invalid points
points = unique(points, 'rows', 'stable');
validIdcs =  points(:,1) ~= 0 & points(:,1) ~= homeSize &...
    points(:,2) ~= 0 & points(:,2) ~= homeSize;
for k = 1:nPoints
    if any(all(transpose(points(k,:) == cell2mat(obstacles'))))
        validIdcs(k) = false;
    end
end
points = points(validIdcs,:);
nPoints = size(points,1);
%% Build adjacency matrix
M = BuildAdjcancyMatrix(points, obstacles, connectionRadius);
%% Get inspection point for each point
[pointsInSight, timeVisVec] = GetPointsInSight(params, points, inspectionPoints, obstacles);
%% Clustering
[clusters, clustersSpectral] = ClusterPoints(points, M);
Ms = zeros(nPoints);
for k = 1:nPoints-1
    k
    for j = k+1:nPoints
        i1 = find(pointsInSight(k,:));
        i2 = find(pointsInSight(j,:));
        in = sum(pointsInSight(k,:) & pointsInSight(j,:));
        un = sum(pointsInSight(k,:) | pointsInSight(j,:));
        if un == 0
            Ms(k,j) = 0;
            Ms(j,k) = 0;
        else
            Ms(k,j) = in/un;
            Ms(j,k) = Ms(k,j);
        end
    end
end
D = diag(sum(Ms));
L = D-Ms;
[V,eigenMat] = eig(L);
kSmallestEigenvectors = V(:,1:nRooms+1);
clustersSpectral = kmeans(kSmallestEigenvectors, nRooms);
%% Plot enviroment
PlotEnvironment(params, points, clusters, M, inspectionPoints, obstacles, 'Clustered with K-Means');
PlotEnvironment(params, points, clustersSpectral, M, inspectionPoints, obstacles, 'Clustered with Spectral Clustering');
if saveEnv
    %% Write text files
    filename = ['syn_' num2str(nRooms) 'rooms'];
    pathToWrite = [outputFolder filename];
    Graph2Text(params, pathToWrite, points, pointsInSight, M, timeVisVec, inspectionPoints, obstacles)
end