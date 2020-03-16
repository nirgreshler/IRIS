function points = PlanarInverseKinematics(G)
nPoints = size(G.graph.Nodes,1);
points = zeros(nPoints,2);
for k = 1:nPoints
    angles = [G.graph.Nodes(k,:).x1 G.graph.Nodes(k,:).x2 G.graph.Nodes(k,:).x3 G.graph.Nodes(k,:).x4 G.graph.Nodes(k,:).x5];
    
    rVec = [cos(angles); sin(angles)];
    pVec = cumsum(rVec,2);
    P = pVec(:,end);
    points(k,:) = P;
end