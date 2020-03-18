function [pathFound, cost] = AStar(points, startIdx, goalIdx, M)

nPoints = size(points,1);
startPoint = points(startIdx, :);
goalPoint = points(goalIdx, :);
closedSet = [];
openSet = [startIdx startPoint];
gScore = inf*ones(nPoints,1);
gScore(startIdx) = 0;
fScore = inf*ones(nPoints,1);
fScore(startIdx) = gScore(startIdx)+h(startPoint, goalPoint);
pathFound = [];
cost = [];
parents = NaN(nPoints,3);

counter = 0;
while ~isempty(openSet)
    counter = counter+1;
    % Get point with minimum cost
    [~, idxInOpenSet] = min(fScore(openSet(:,1)));
    currentPoint = openSet(idxInOpenSet,:);
    curIdx = currentPoint(1);
    
    if all(currentPoint(2:3)==goalPoint)
        pathFound = ReconstructPath(parents, [goalIdx goalPoint]);
        cost = sum(sqrt(sum(diff(pathFound(:,2:3)).^2,2)));
        return
    end
    
    % Remove point from open
    openSet(idxInOpenSet,:) = [];
    
    % Add point to closed
    closedSet = [closedSet; currentPoint];
    
    % Get neighbors
    neighbors = find(M(curIdx,:) > 0);
    for n = 1:length(neighbors)
        neighborIdx = neighbors(n);
        neighbor = points(neighborIdx,:);
        curG = gScore(curIdx)+M(curIdx, neighborIdx);
        if ~isempty(closedSet)
            idxInClosed = find(closedSet(1,:) == neighborIdx);
            if ~isempty(idxInClosed) || curG >= gScore(neighborIdx)
                continue
            end
        end
        if ~isempty(openSet)
            neighborIdxInOpen = find(openSet(1,:) == neighborIdx);
        else
            neighborIdxInOpen = [];
        end
        if ~isempty(neighborIdxInOpen) || curG < gScore(neighborIdx)
            parents(neighborIdx,:) = currentPoint;
            gScore(neighborIdx) = curG;
            fScore(neighborIdx) = gScore(neighborIdx)+h(neighbor, goalPoint);
            if isempty(neighborIdxInOpen)
                openSet = [openSet; [neighborIdx neighbor]];
            end
        end
    end
end

function path = ReconstructPath(cameFrom, goalPoint)
parent = cameFrom(goalPoint(1),:);
if ~isnan(parent(2))
    subpath = ReconstructPath(cameFrom, parent);
    path = [subpath; goalPoint];
else
    path = goalPoint;
end
