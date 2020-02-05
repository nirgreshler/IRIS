clear
close all
clc
nSqrRooms = 4;
homeSize = 16;%nSqrRooms^2;
nRooms = nSqrRooms^2;
envGrid = 0.01;
pointsGrid = 0.1;
doorSize = 0.2;
nPoints = 500;
connectionRadius = 3;
samplingMethod = 'Uniform'; % 'Uniform' / 'Sobol'
fol = fileparts(mfilename('fullpath'));

outputFolder = fullfile(fol, '..', 'Graphs', filesep);
%% Create points
inspectionPoints = GetInspectionPoints(homeSize, pointsGrid);
[obstacles, doors] = GetObstacles(homeSize, nRooms, doorSize, envGrid);
skip = randi(100000);
switch samplingMethod
    case 'Uniform'
        points = envGrid*round(rand(nPoints,2)*homeSize/envGrid);
    case 'Sobol'
        sampler = sobolset(2);
        sampler.Skip = skip;
        points = sampler.net(nPoints)*homeSize;
end
%% Remove invalid points
points = unique(points, 'rows');
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
M = zeros(nPoints); 
for k = 1:nPoints
    p1 = points(k,:);
    for m = k+1:nPoints
        p2 = points(m,:);
        if norm(p1-p2) <= connectionRadius && CollisionDetector(p1, p2, obstacles)
            M(k,m) = 1;
            M(m,k) = 1;
        end
    end
end
edges = M2Edges(M);
Mtag = Edges2M(edges);
%% Get inspection point for each point
pointsInSight = zeros(nPoints, size(inspectionPoints,1));
for k = 1:nPoints
    for p = 1:size(inspectionPoints,1)
        if CollisionDetector(points(k,:), inspectionPoints(p,:), obstacles)
            pointsInSight(k,p) = 1;
        end
    end
end
%% Clustering
[clusters, clustersSpectral] = ClusterPoints(points, M, nRooms);
%% Plot enviroment
PlotEnvironment(points, clusters, M, inspectionPoints, obstacles, homeSize, 'Clustered with K-Means');
PlotEnvironment(points, clustersSpectral, M, inspectionPoints, obstacles, homeSize, 'Clustered with Spectral Clustering');
%% Write text files
filename = ['syn_' num2str(nRooms) 'rooms'];
fId = fopen([outputFolder filename '_conf'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f\n', k-1, points(k,1), points(k,2));
end
fclose(fId);

fId = fopen([outputFolder filename '_vertex'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d 0 0 ', k-1);
    fprintf(fId, strrep(strrep(num2str(find(pointsInSight(k,:))), '   ', ' '), '  ', ' '));
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