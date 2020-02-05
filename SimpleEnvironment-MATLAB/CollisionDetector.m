function isFree = CollisionDetector(p1, p2, doors, envGrid, doorSize)
if p1(1) > p2(1)
   tmp = p2;
   p2 = p1;
   p1 = tmp;
end
isFree = true;
halfRoomSize = doors(1,2);
a = (p2(2)-p1(2))/(p2(1)-p1(1));
if isinf(a)
    if p1(1) < doors(1,1) || (p1(1) > doors(1,1)+doorSize && p1(1) < doors(2,1))...
            || p1(1) > doors(2,1)+1
        isFree = false;
        return
    end
end
if a == 0
    if p1(2) < doors(3,2) || (p1(2) > doors(3,2)+doorSize && p1(2) < doors(4,2))...
            || p1(2) > doors(4,2)+1
        isFree = false;
        return
    end  
end
b = p1(2)-a*p1(1);
xVec = p1(1):envGrid:p2(1);
yVec = a*xVec+b;
xHrs = (halfRoomSize-b)/a;
[m,i] = min(abs(yVec-halfRoomSize));
if xHrs >= p1(1) && xHrs <= p2(1)
    if xHrs < doors(1,1) || (xHrs > doors(1,1)+doorSize && xHrs < doors(2,1))...
            || xHrs > doors(2,1)+doorSize
        isFree = false;
        return
    end   
end
yHrs = a*halfRoomSize+b;
if yHrs >= min(p1(2), p2(2)) && yHrs <= max(p1(2), p2(2))
    if yHrs < doors(3,2) || (yHrs > doors(3,2)+doorSize && yHrs < doors(4,2))...
            || yHrs > doors(4,2)+doorSize
        isFree = false;
        return
    end   
end

