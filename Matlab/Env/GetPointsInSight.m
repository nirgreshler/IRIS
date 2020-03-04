function [pointsInSight, timeVisVec] = GetPointsInSight(params, points, inspectionPoints, obstacles)

nPoints = size(points,1);
pointsInSight = zeros(nPoints, size(inspectionPoints,1));
timeVisVec = zeros(nPoints,1);
for k = 1:nPoints
    tic
    for p = 1:size(inspectionPoints,1)
        if CollisionDetector(points(k,:), inspectionPoints(p,:), obstacles, params.sightRadius)
            pointsInSight(k,p) = 1;
        end
    end
    timeVisVec(k) = toc;
end