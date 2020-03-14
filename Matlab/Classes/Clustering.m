classdef Clustering < handle
    properties
        type
        params
    end
    
    methods
        function c = Clustering(type, params)
            c.type = type;
            c.params = params;
        end
        
        function clusters = cluster(c, conf, M)           
            switch c.type
                case 'kmeans'
                    clusters = KMeansClustering(c.params, conf, c.params.k);
                case 'spectral'
                    clusters = SpectralClustering(c.params, conf, M);
                case 'inspection'
                    pointsInSight = GetPointsInSight(c.params, conf, c.params.inspectionPoints, c.params.obstacles);
                    clusters = InspectionClustering(c.params, conf, pointsInSight);
            end
            
        end
    end
    
end