function PlotEnvironment(points, clusters, M, inspectionPoints, obstacles, roomSize, plotTitle)
if ~exist('plotTitle', 'var') || isempty(plotTitle)
    plotTitle = 'Environment';
end

nPoints = size(points,1);
nClusters = length(unique(clusters));
figure; hold all; axis equal

% outside walls
plot([0 roomSize], [0 0], '-k', 'LineWidth', 1)
plot([roomSize roomSize], [0 roomSize], '-k', 'LineWidth', 1)
plot([0 roomSize], [roomSize roomSize], '-k', 'LineWidth', 1)
plot([0 0], [0 roomSize], '-k', 'LineWidth', 1)

% inspection points
plot(inspectionPoints(:,1), inspectionPoints(:,2), 'ob')

% obstacles
plot(obstacles{1}(:,1), obstacles{1}(:,2), '.k', 'LineWidth', 1)
plot(obstacles{2}(:,1), obstacles{2}(:,2), '.k', 'LineWidth', 1)

% edges
for k = 1:nPoints
    for m = k+1:nPoints
        if M(k,m) == 1
            p1 = points(k,:);
            p2 = points(m,:);
            plot([p1(1) p2(1)], [p1(2) p2(2)], '--k')
        end
    end
end

% points
colorOrder = get(gca, 'ColorOrder');
for k = 1:nClusters
    plot(points(clusters == k,1), points(clusters == k,2), '.', 'Color', colorOrder(k,:), 'MarkerSize', 15, 'LineWidth', 3)
end

title(plotTitle)