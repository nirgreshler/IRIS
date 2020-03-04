clear
close all
clc
nSqrRooms = 3;
homeSize = 9;%nSqrRooms^2;
nRooms = nSqrRooms^2;
roomSize = homeSize/nSqrRooms;
envGrid = 0.01;
doorSize = 0.3;
nPoints = 2000;
nInspectionPoints = 400;
connectionRadius = 1;
sightRadius = roomSize*sqrt(2)/2;
samplingMethod = 'RRT'; % 'Uniform' / 'Sobol' / 'RRT'
fol = fileparts(mfilename('fullpath'));

outputFolder = fullfile(fol, '..', 'Graphs', filesep);
saveEnv = true;
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
        startPoint = [1e-3 1e-3]; %envGrid*round(rand(1,2)*homeSize/envGrid);
        points = startPoint;
        for k = 1:nPoints
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
pointsInSight = zeros(nPoints, size(inspectionPoints,1));
timeVisVec = zeros(nPoints,1);
for k = 1:nPoints
    tic
    for p = 1:size(inspectionPoints,1)
        if CollisionDetector(points(k,:), inspectionPoints(p,:), obstacles, sightRadius)
            pointsInSight(k,p) = 1;
        end
    end
    timeVisVec(k) = toc;
end
%% Clustering
[clusters, clustersSpectral] = ClusterPoints(points, M, nRooms);
%% Plot enviroment
PlotEnvironment(points, clusters, M, inspectionPoints, obstacles, homeSize, 'Clustered with K-Means');
PlotEnvironment(points, clustersSpectral, M, inspectionPoints, obstacles, homeSize, 'Clustered with Spectral Clustering');
if saveEnv
    %% Write text files
    filename = ['syn_' num2str(nRooms) 'rooms'];
    fId = fopen([outputFolder filename '_conf'], 'w');
    for k = 1:nPoints
        fprintf(fId, '%d %f %f\n', k-1, points(k,1), points(k,2));
    end
    fclose(fId);
    
    fId = fopen([outputFolder filename '_vertex'], 'w');
    for k = 1:nPoints
        fprintf(fId, '%d %f 0 ', k-1, timeVisVec(k)*1e3);
        fprintf(fId, strrep(strrep(num2str(find(pointsInSight(k,:))-1), '   ', ' '), '  ', ' '));
        fprintf(fId, '\n');
    end
    fclose(fId);
    
    fId = fopen([outputFolder filename '_edge'], 'w');
    for k = 1:nPoints
        for p = k+1:nPoints
            if M(k,p) == 1
                fprintf(fId, '%d %d 1 1 0 0 %f\n', k-1, p-1, norm(points(k,:)-points(p,:)));
            end
        end
    end
    fclose(fId);
    
    fId = fopen([outputFolder filename '_inspectionPoints'], 'w');
    for k = 1:size(inspectionPoints,1)
        fprintf(fId, '%d %f %f\n', k-1, inspectionPoints(k,1), inspectionPoints(k,2));
    end
    fclose(fId);
    
    fId = fopen([outputFolder filename '_obstacles'], 'w');
    for k = 1:numel(obstacles)
        for p = 1:size(obstacles{k},1)
            fprintf(fId, '%d %f %f\n', k-1, obstacles{k}(p,1), obstacles{k}(p,2));
        end
    end
    fclose(fId);
    
    fId = fopen([outputFolder filename '_params'], 'w');
    fprintf(fId, 'homeSize: %d\n', homeSize);
    fprintf(fId, 'doorSize: %f\n', doorSize);
    fprintf(fId, 'connectionRadius: %f\n', connectionRadius);
    fprintf(fId, 'sightRadius: %f\n', sightRadius);
    fclose(fId);
end