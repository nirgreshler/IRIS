classdef Edge < handle

properties
    source Vertex
    target Vertex
    checked
    valid
    time_forward_kinematics
    time_collision_detection
    cost
end

methods
    function e = Edge(G, line)
        e.source = G.getNodeById(line(1));
        e.target = G.getNodeById(line(2));
        
        e.checked = line(3);
        e.valid = line(4);
        e.time_forward_kinematics = line(5);
        e.time_collision_detection = line(6);
        e.cost = line(7);
    end
end

end