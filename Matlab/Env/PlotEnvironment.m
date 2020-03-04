function PlotEnvironment(params, points, clusters, M, inspectionPoints, obstacles, plotTitle)

homeSize = params.homeSize;
if ~isfield(params, 'plotEdges')
    params.plotEdges = false;
end

if ~exist('plotTitle', 'var') || isempty(plotTitle)
    plotTitle = 'Environment';
end

nPoints = size(points,1);
if isempty(clusters)
    clusters = ones(nPoints,1);
end
nClusters = length(unique(clusters));
plotTitle = [plotTitle, '(', num2str(nClusters), ' Clusters)'];
figure; hold all; axis equal

% outside walls
plot([0 homeSize], [0 0], '-k', 'LineWidth', 1)
plot([homeSize homeSize], [0 homeSize], '-k', 'LineWidth', 1)
plot([0 homeSize], [homeSize homeSize], '-k', 'LineWidth', 1)
plot([0 0], [0 homeSize], '-k', 'LineWidth', 1)

% inspection points
plot(inspectionPoints(:,1), inspectionPoints(:,2), 'ob')

% obstacles
for k = 1:numel(obstacles)
    plot(obstacles{k}(:,1), obstacles{k}(:,2), '.k', 'LineWidth', 1)
end

if params.plotEdges
    % edges
    for k = 1:nPoints
        for m = k+1:nPoints
            if M(k,m) > 0
                p1 = points(k,:);
                p2 = points(m,:);
                plot([p1(1) p2(1)], [p1(2) p2(2)], '--k')
            end
        end
    end
end

% points
colorOrder = linspecer(nClusters);
nColors = size(colorOrder,1);
for k = 1:nClusters
    color = colorOrder(mod(k-1,nColors)+1,:);
    plot(points(clusters == k,1), points(clusters == k,2), '.', 'Color', color, 'MarkerSize', 15, 'LineWidth', 3)
end

title(plotTitle)