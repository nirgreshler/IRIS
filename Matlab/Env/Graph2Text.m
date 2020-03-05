function Graph2Text(params, pathToWrite, points, pointsInSight, M, timeVisVec, inspectionPoints, obstacles)
nPoints = size(points,1);

if ~exist('timeVisVec', 'var') || isempty(timeVisVec)
    timeVisVec = zeros(nPoints,1);
end

fId = fopen([pathToWrite '_params'], 'w');
fprintf(fId, 'homeSize: %d\n', params.homeSize);
fprintf(fId, 'doorSize: %f\n', params.doorSize);
fprintf(fId, 'connectionRadius: %f\n', params.connectionRadius);
fprintf(fId, 'sightRadius: %f\n', params.sightRadius);
fprintf(fId, 'nRooms: %f\n', params.nRooms);
fclose(fId);

fId = fopen([pathToWrite '_conf'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f\n', k-1, points(k,1), points(k,2));
end
fclose(fId);

fId = fopen([pathToWrite '_vertex'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f 0 ', k-1, timeVisVec(k)*1e3);
    fprintf(fId, strrep(strrep(num2str(find(pointsInSight(k,:))-1), '   ', ' '), '  ', ' '));
    fprintf(fId, '\n');
end
fclose(fId);

fId = fopen([pathToWrite '_edge'], 'w');
for k = 1:nPoints
    for p = k+1:nPoints
        if M(k,p) > 0
            fprintf(fId, '%d %d 1 1 0 0 %f\n', k-1, p-1, norm(points(k,:)-points(p,:)));
        end
    end
end
fclose(fId);

if exist('inspectionPoints', 'var') && ~isempty(inspectionPoints)
    fId = fopen([pathToWrite '_inspectionPoints'], 'w');
    for k = 1:size(inspectionPoints,1)
        fprintf(fId, '%d %f %f\n', k-1, inspectionPoints(k,1), inspectionPoints(k,2));
    end
    fclose(fId);
end

if exist('obstacles', 'var') && ~isempty(obstacles)
    fId = fopen([pathToWrite '_obstacles'], 'w');
    for k = 1:numel(obstacles)
        for p = 1:size(obstacles{k},1)
            fprintf(fId, '%d %f %f\n', k-1, obstacles{k}(p,1), obstacles{k}(p,2));
        end
    end
    fclose(fId);
end