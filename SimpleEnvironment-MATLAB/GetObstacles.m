function [obstacles, doors] = GetObstacles(roomSize, doorSize, envGrid)
% obstacles are walls with doors in the middle of the room (cell array)

nPointsInObstacle = (roomSize/2-doorSize)/envGrid;
% Horizontal wall
doorLocation1 = envGrid*round(rand*(roomSize/2-doorSize)/envGrid);
doorLocation2 = envGrid*round((roomSize/2+rand*(roomSize/2-doorSize))/envGrid);
doors = [doorLocation1 roomSize/2; doorLocation2 roomSize/2];


obstacles{1} = [transpose([0:envGrid:doorLocation1-envGrid (doorLocation1+doorSize+envGrid):envGrid:roomSize/2]) roomSize/2*ones(nPointsInObstacle,1)];
obstacles{1} = [obstacles{1}; [transpose([roomSize/2+envGrid:envGrid:doorLocation2-envGrid (doorLocation2+doorSize+envGrid):envGrid:roomSize]) roomSize/2*ones(nPointsInObstacle-1,1)]];

% Vertical wall
doorLocation1 = envGrid*round(rand*(roomSize/2-doorSize)/envGrid);
doorLocation2 = envGrid*round((roomSize/2+rand*(roomSize/2-doorSize))/envGrid);
doors = [doors; roomSize/2 doorLocation1; roomSize/2 doorLocation2];

obstacles{2} = [roomSize/2*ones(nPointsInObstacle,1) transpose([0:envGrid:doorLocation1-envGrid (doorLocation1+doorSize+envGrid):envGrid:roomSize/2])];
obstacles{2} = [obstacles{2}; [roomSize/2*ones(nPointsInObstacle-1,1) transpose([roomSize/2+envGrid:envGrid:doorLocation2-envGrid (doorLocation2+doorSize+envGrid):envGrid:roomSize])]];
