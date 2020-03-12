classdef Cluster < handle

properties
    id
    v Vertex
end

methods
    function c = Cluster(id, vertices)
        c.id = id;
        c.v = vertices;
    end
end

end