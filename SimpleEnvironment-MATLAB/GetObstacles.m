function [obstacles, doors] = GetObstacles(homeSize, nRooms, doorSize, envGrid)
% obstacles are walls with doors in home (cell array)
wallsLocationRatio = 1/sqrt(nRooms);
nWalls = 1/wallsLocationRatio-1;
nDoors = 1/wallsLocationRatio;

nPointsInObstacle = (homeSize*wallsLocationRatio-doorSize)/envGrid;

doors = [];
obstacles = [];
% Horizontal walls
for p = 1:nWalls
    yCord = wallsLocationRatio*p*homeSize;
    for k = 1:nDoors
        boundaries = [(k-1)*wallsLocationRatio*homeSize k*wallsLocationRatio*homeSize];
        doorLocation = envGrid*round((rand*(diff(boundaries)-doorSize)+boundaries(1))/envGrid);
        doors(end+1,:) = [doorLocation yCord];
        xCords = transpose(boundaries(1):envGrid:doorLocation-envGrid);
        obstacles{end+1} = [xCords yCord*ones(size(xCords))];
        xCords = transpose((doorLocation+doorSize):envGrid:boundaries(2));
        obstacles{end+1} = [xCords yCord*ones(size(xCords))];
    end
end

% Vertical walls
for p = 1:nWalls
    xCord = wallsLocationRatio*p*homeSize;
    for k = 1:nDoors
        boundaries = [(k-1)*wallsLocationRatio*homeSize k*wallsLocationRatio*homeSize];
        doorLocation = envGrid*round((rand*(diff(boundaries)-doorSize)+boundaries(1))/envGrid);
        doors(end+1,:) = [xCord, doorLocation];
        yCords = transpose(boundaries(1):envGrid:doorLocation-envGrid);
        obstacles{end+1} = [xCord*ones(size(yCords)) yCords];
        yCords = transpose((doorLocation+doorSize):envGrid:boundaries(2));
        obstacles{end+1} = [xCord*ones(size(yCords)) yCords];
    end
end