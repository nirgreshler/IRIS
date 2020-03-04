function M = BuildAdjcancyMatrix(points, obstacles, connectionRadius)
nPoints = size(points,1);
M = zeros(nPoints);
for k = 1:nPoints
    p1 = points(k,:);
    for m = k+1:nPoints
        p2 = points(m,:);
        if CollisionDetector(p1, p2, obstacles, connectionRadius)
            M(k,m) = 1;
            M(m,k) = 1;
        end
    end
    disp(['Building Adjancy Matrix: ', num2str(k) '/' num2str(nPoints)]);
end