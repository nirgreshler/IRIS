function PlotBridgeEnvironment(params, G, inspectionPoints, plotTitle)

if ~exist('plotTitle', 'var') || isempty(plotTitle)
    plotTitle = 'Bridge Environment';
end
if ~isfield(params, 'plotEdges')
    params.plotEdges = false;
end
if ~isfield(params, 'inspectInspection')
    params.inspectInspection = false;
end
if ~isfield(params, 'showText')
    params.showText = false;
end
points = [G.graph.Nodes.x1, G.graph.Nodes.x2, G.graph.Nodes.x3];
clusters = G.graph.Nodes.cluster;
    
graphHandles = [];

nPoints = size(points,1);
if isempty(clusters)
    clusters = ones(nPoints,1);
end
clusterIdcs = unique(clusters);
nClusters = length(clusterIdcs);
plotTitle = [plotTitle, ' (', num2str(nClusters), ' Clusters)'];
f = figure; hold all; 
axis equal

% inspection points
plot3(inspectionPoints(:,1), inspectionPoints(:,2), inspectionPoints(:,3), 'ob')

% points
colorOrder = linspecer(nClusters);
nColors = size(colorOrder,1);
for k = 1:nClusters
    color = colorOrder(mod(k-1,nColors)+1,:);
    pH = plot3(points(clusters == clusterIdcs(k),1), points(clusters == clusterIdcs(k),2), points(clusters == clusterIdcs(k),3), '.', 'Color', color, 'MarkerSize', 15, 'LineWidth', 3);
    pH.ButtonDownFcn = {@showInsPoints, points};
    graphHandles = [graphHandles pH];
end

colorOrder = linspecer(nClusters);
for k = 1:nClusters
    idcs = G.graph.Nodes.cluster == k;
    color = colorOrder(k,:);
    plot3(G.graph.Nodes.x1(idcs), G.graph.Nodes.x2(idcs), G.graph.Nodes.x3(idcs), '.', 'Color', color, 'MarkerSize', 15, 'LineWidth', 3);
end

if params.showText
    for i = 1:nPoints
        graphHandles = [graphHandles text(points(i, 1), points(i, 2), num2str(i-1), 'Color', 'r')];
    end
end

f.ButtonDownFcn = {@(f, ~, h) toggleGraph(f, graphHandles), graphHandles};

title(plotTitle)
xlabel('x'); ylabel('y'); zlabel('z');

if params.inspectInspection
    disp('Right click to exit')
    while 1
        [x, y, button] = ginput(1);
        if button == 3
            break
        end
        if exist('hInspec', 'var')
            delete(hInspec)
            delete(hInspected)
        end
        [~,i] = min(sqrt(sum(([x y]-points).^2,2)));
        pointToCheck = points(i,:);
        pointsInSight = logical(GetPointsInSight(params, pointToCheck, inspectionPoints, obstacles));
        hInspec = plot(inspectionPoints(pointsInSight,1), inspectionPoints(pointsInSight,2), 'oy');
        hInspected = plot(pointToCheck(1), pointToCheck(2), 'ok', 'LineWidth', 2);
    end
end


    function toggleGraph(f, h)
        if strcmp(h(1).Visible, 'on')
            set(h, 'Visible', 'off');
        else
            set(h, 'Visible', 'on');
        end
    end

    function showInsPoints(f, e ,ps)
        xx = e.IntersectionPoint(1);
        yy = e.IntersectionPoint(2);
        [~,ii] = min(sqrt(sum(([xx yy]-ps).^2,2)));
        if exist('hInspec', 'var')
            delete(hInspec)
            delete(hInspected)
        end
        pointToCheck = ps(ii,:);
        pointsInSight = logical(GetPointsInSight(params, pointToCheck, inspectionPoints, obstacles));
        hInspec = plot(inspectionPoints(pointsInSight,1), inspectionPoints(pointsInSight,2), 'oy');
        hInspected = plot(pointToCheck(1), pointToCheck(2), 'ok', 'LineWidth', 2);
    end


end

