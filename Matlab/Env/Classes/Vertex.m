classdef Vertex < handle

properties
    id
    time_vis
    time_build
    vis
    
    conf
end

methods
    function v = Vertex(c_line, v_line)
        assert(c_line(1) == v_line(1));
        v.id = c_line(1);
        
        v.conf = c_line(2:end);
        v.time_vis = v_line(2);
        v.time_build = v_line(3);
        
        vis_line = v_line(4:end);
        v.vis = vis_line(~isnan(vis_line));
    end
end

end