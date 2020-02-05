function inspectionPoints = GetInspectionPoints(homeSize, pointsGrid)
% Inspection points are on the boundaries of the home
nPointsAlongWall = homeSize/pointsGrid;
inspectionPoints = [transpose(0:pointsGrid:homeSize-pointsGrid) zeros(nPointsAlongWall,1)];
inspectionPoints = [inspectionPoints; [homeSize*ones(nPointsAlongWall,1) transpose(0:pointsGrid:homeSize-pointsGrid)]];
inspectionPoints = [inspectionPoints; [transpose(fliplr(pointsGrid:pointsGrid:homeSize)) homeSize*ones(nPointsAlongWall,1)]];
inspectionPoints = [inspectionPoints; [zeros(nPointsAlongWall,1) transpose(fliplr(pointsGrid:pointsGrid:homeSize))]];