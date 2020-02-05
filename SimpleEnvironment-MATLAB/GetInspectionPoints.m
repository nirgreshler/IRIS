function inspectionPoints = GetInspectionPoints(roomSize, pointsGrid)
% Inspection points are on the boundaries of the room
nPointsAlongWall = roomSize/pointsGrid;
inspectionPoints = [transpose(0:pointsGrid:roomSize-pointsGrid) zeros(nPointsAlongWall,1)];
inspectionPoints = [inspectionPoints; [roomSize*ones(nPointsAlongWall,1) transpose(0:pointsGrid:roomSize-pointsGrid)]];
inspectionPoints = [inspectionPoints; [transpose(fliplr(pointsGrid:pointsGrid:roomSize)) roomSize*ones(nPointsAlongWall,1)]];
inspectionPoints = [inspectionPoints; [zeros(nPointsAlongWall,1) transpose(fliplr(pointsGrid:pointsGrid:roomSize))]];