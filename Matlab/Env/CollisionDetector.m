function isFree = CollisionDetector(p1, p2, obstacles)
if p1(1) > p2(1)
   tmp = p2;
   p2 = p1;
   p1 = tmp;
end
isFree = true;
a = (p2(2)-p1(2))/(p2(1)-p1(1));
b = p1(2)-a*p1(1);
for k = 1:numel(obstacles)
    obstacle = obstacles{k};
    aObs = (obstacle(end,2)-obstacle(1,2))/(obstacle(end,1)-obstacle(1,1));
    if aObs == 0
        bObs = obstacle(1,2)-aObs*obstacle(1,1);
        xIntersection = (bObs-b)/(a-aObs);
        if xIntersection >= obstacle(1,1) && xIntersection <= obstacle(end,1)...
            && xIntersection >= p1(1) && xIntersection <= p2(1)
            isFree = false;
            return
        end
    elseif isinf(aObs)
        yIntersection = a*obstacle(1,1)+b;
        if yIntersection >= obstacle(1,2) && yIntersection <= obstacle(end,2)...
                && yIntersection >= min(p1(2), p2(2)) && yIntersection <= max(p1(2), p2(2))
            isFree = false;
            return
        end
    end
end