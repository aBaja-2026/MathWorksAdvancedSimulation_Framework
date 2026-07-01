function varargout = selectRRHDRoadPoints(rrhdFile, distanceMeters, randomSeed, options)
% selectRRHDRoadPoints Pick a random left-side road point and a point 500 m away.
%
% The selected point A is placed inside the left half of a non-junction
% driving lane. Point B is found by walking the drivable lane topology by
% arclength. Both A and B are rejected if they land on junction lanes.
%
% Usage:
%   result = selectRRHDRoadPoints([], 500, 42)
%   [pointA, pointB] = selectRRHDRoadPoints([], 500, 42)
%   [pointA, pointB, result] = selectRRHDRoadPoints([], 500, 42)
%   [pointA, pointB] = selectRRHDRoadPoints([], 500, 42, ...
%       struct("Verbose", false, "CreatePlot", false, "SaveResult", false, ...
%       "Fast", true, "MapEdgeMargin", 50, "StartStationRandomWindow", 60))

if nargin < 1 || isempty(rrhdFile) || strlength(string(rrhdFile)) == 0
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    rrhdFile = fullfile(repoRoot, "localFolder", "IndianCityBlock.rrhd");
else
    rrhdFile = char(rrhdFile);
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
end

if nargin < 2 || isempty(distanceMeters)
    distanceMeters = 500;
end

if nargin < 3 || isempty(randomSeed)
    rng("shuffle");
else
    rng(randomSeed);
end

if nargin < 4 || isempty(options)
    options = struct;
end
verbose = getOption(options, "Verbose", true);
createPlot = getOption(options, "CreatePlot", true);
saveResult = getOption(options, "SaveResult", true);
fast = getOption(options, "Fast", false);
verticalStartOnly = getOption(options, "VerticalStartOnly", false);
mapEdgeMargin = getOption(options, "MapEdgeMargin", 80);
startStationRandomWindow = getOption(options, "StartStationRandomWindow", 5);
startStationAttempts = getOption(options, "StartStationAttempts", 1);

if ~isfile(rrhdFile)
    error("selectRRHDRoadPoints:FileNotFound", "RRHD file not found: %s", rrhdFile);
end

outDir = fullfile(repoRoot, "analysis_outputs");
if ~isfolder(outDir)
    mkdir(outDir);
end

rrMap = roadrunnerHDMap;
read(rrMap, rrhdFile);

lanes = rrMap.Lanes;
laneMap = makeLaneMap(lanes);
boundaryMap = makeBoundaryMap(rrMap.LaneBoundaries);
junctionLaneIds = collectJunctionLaneIds(rrMap.Junctions);

[candidateIdx, candidateDir] = findCandidateStartLanes(lanes, boundaryMap, junctionLaneIds);
if isempty(candidateIdx)
    error("selectRRHDRoadPoints:NoCandidates", ...
        "No non-junction driving lanes with usable left-side geometry were found.");
end
if verticalStartOnly
    [verticalIdx, verticalDir] = filterVerticalStartLanes(lanes, candidateIdx, candidateDir);
    if ~isempty(verticalIdx)
        candidateIdx = verticalIdx;
        candidateDir = verticalDir;
    end
end
if verbose
    fprintf("Candidate non-junction driving lanes: %d\n", numel(candidateIdx));
end
mapBounds = drivingLaneBounds(lanes);

minEndpointSeparation = min(0.10 * distanceMeters, 75);
[success, route, diagnostics] = findRandomQualityRoute( ...
    lanes, laneMap, boundaryMap, junctionLaneIds, candidateIdx, candidateDir, ...
    distanceMeters, minEndpointSeparation, mapBounds, fast, mapEdgeMargin, ...
    startStationRandomWindow, startStationAttempts);

if ~success
    if verbose
        fprintf("Raw 500 m routes found before quality filtering: %d\n", diagnostics.RawRouteCount);
        fprintf("Best endpoint separation: %.3f m; polyline length error: %.3f m\n", ...
            diagnostics.BestEndpointSeparation, diagnostics.BestPolylineError);
    end
    error("selectRRHDRoadPoints:NoRoute", ...
        "Could not find a visually valid non-junction point B %.1f m away.", ...
        distanceMeters);
end

result = struct;
result.File = string(rrhdFile);
result.DistanceMetersRequested = distanceMeters;
result.DistanceMetersMeasured = route.distance;
result.PointA = route.pointA;
result.PointB = route.pointB;
result.StartLaneID = string(lanes(route.startIdx).ID);
result.EndLaneID = string(lanes(route.endIdx).ID);
result.StartTravelDirection = directionText(route.startDir);
result.EndTravelDirection = directionText(route.endDir);
result.RouteLaneIDs = route.laneIds;
result.RoutePolyline = route.points;

plotFile = "";
matFile = "";
if createPlot
    plotFile = fullfile(outDir, "IndianCityBlock_rrhd_random_500m_points.png");
    plotSelection(rrMap, result, plotFile);
end
if saveResult
    matFile = fullfile(outDir, "IndianCityBlock_rrhd_random_500m_points.mat");
    save(matFile, "result");
end
result.PlotFile = string(plotFile);
result.ResultFile = string(matFile);

if verbose
    fprintf("\nSelected RRHD road points\n");
    fprintf("  A: [%.3f, %.3f, %.3f]\n", result.PointA);
    fprintf("  B: [%.3f, %.3f, %.3f]\n", result.PointB);
    fprintf("  Route distance: %.6f m\n", result.DistanceMetersMeasured);
    fprintf("  Start lane: %s (%s)\n", result.StartLaneID, result.StartTravelDirection);
    fprintf("  End lane:   %s (%s)\n", result.EndLaneID, result.EndTravelDirection);
    if createPlot
        fprintf("  Plot:       %s\n", plotFile);
    end
    if saveResult
        fprintf("  Result MAT: %s\n", matFile);
    end
end

if nargout <= 1
    varargout = {result};
elseif nargout == 2
    varargout = {result.PointA, result.PointB};
elseif nargout == 3
    varargout = {result.PointA, result.PointB, result};
else
    error("selectRRHDRoadPoints:TooManyOutputs", ...
        "Use one output for the result struct, two outputs for pointA and pointB, or three outputs for pointA, pointB, and result.");
end
end

function value = getOption(options, name, defaultValue)
if isstruct(options) && isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function laneMap = makeLaneMap(lanes)
laneMap = containers.Map("KeyType", "char", "ValueType", "double");
for i = 1:numel(lanes)
    laneMap(char(lanes(i).ID)) = i;
end
end

function boundaryMap = makeBoundaryMap(boundaries)
boundaryMap = containers.Map("KeyType", "char", "ValueType", "any");
for i = 1:numel(boundaries)
    boundaryMap(char(boundaries(i).ID)) = boundaries(i).Geometry;
end
end

function ids = collectJunctionLaneIds(junctions)
ids = strings(0, 1);
for i = 1:numel(junctions)
    refs = junctions(i).Lanes;
    for j = 1:numel(refs)
        ids(end + 1, 1) = string(refs(j).ID); %#ok<AGROW>
    end
end
ids = unique(ids);
end

function tf = isJunctionLane(lane, junctionLaneIds)
tf = any(junctionLaneIds == string(lane.ID));
end

function [idx, dirSign] = findCandidateStartLanes(lanes, boundaryMap, junctionLaneIds)
idx = zeros(0, 1);
dirSign = zeros(0, 1);

for i = 1:numel(lanes)
    lane = lanes(i);
    if string(lane.LaneType) ~= "Driving" || isJunctionLane(lane, junctionLaneIds)
        continue
    end

    dir = laneDirectionSign(lane);
    if dir == 0
        continue
    end

    try
        geom = leftSideRoadPolyline(lane, dir, boundaryMap);
        if polylineLength(geom) >= 15
            idx(end + 1, 1) = i; %#ok<AGROW>
            dirSign(end + 1, 1) = dir; %#ok<AGROW>
        end
    catch
        % Skip lanes with incomplete boundary references.
    end
end
end

function [idx, dirSign] = filterVerticalStartLanes(lanes, candidateIdx, candidateDir)
idx = zeros(0, 1);
dirSign = zeros(0, 1);
for i = 1:numel(candidateIdx)
    lane = lanes(candidateIdx(i));
    yawDeg = laneStartYaw(lane, candidateDir(i));
    if abs(sind(yawDeg)) >= 0.90
        idx(end + 1, 1) = candidateIdx(i); %#ok<AGROW>
        dirSign(end + 1, 1) = candidateDir(i); %#ok<AGROW>
    end
end
end

function yawDeg = laneStartYaw(lane, dirSign)
geom = travelLanePolyline(lane, dirSign);
if size(geom, 1) < 2
    yawDeg = NaN;
    return
end
delta = geom(2, 1:2) - geom(1, 1:2);
yawDeg = atan2d(delta(2), delta(1));
end

function dir = laneDirectionSign(lane)
travelDirection = string(lane.TravelDirection);
if travelDirection == "Forward"
    dir = 1;
elseif travelDirection == "Backward"
    dir = -1;
else
    dir = 0;
end
end

function text = directionText(dir)
if dir == 1
    text = "Forward";
elseif dir == -1
    text = "Backward";
else
    text = "Undirected";
end
end

function geom = travelLanePolyline(lane, dirSign)
geom = lane.Geometry;
if isempty(geom)
    error("selectRRHDRoadPoints:EmptyLane", "Lane has no geometry points.");
end

if dirSign == -1 && size(geom, 1) > 1
    geom = flipud(geom);
end
end

function geom = leftSideRoadPolyline(lane, dirSign, boundaryMap)
centerGeom = lane.Geometry;
if size(centerGeom, 1) < 2
    error("selectRRHDRoadPoints:ShortLane", "Lane has fewer than two geometry points.");
end

if dirSign == 1
    sideRef = lane.LeftLaneBoundary;
else
    sideRef = lane.RightLaneBoundary;
end

sideGeom = alignedBoundaryGeometry(sideRef, boundaryMap);
centerDist = cumulativeDistance(centerGeom);
if centerDist(end) <= eps
    error("selectRRHDRoadPoints:ZeroLengthLane", "Lane has zero-length geometry.");
end

sidePts = interpolatePolylineAtFractions(sideGeom, centerDist ./ centerDist(end));
leftSideFraction = 0.50;
geom = centerGeom + leftSideFraction .* (sidePts - centerGeom);

if dirSign == -1
    geom = flipud(geom);
end
end

function geom = alignedBoundaryGeometry(alignedRef, boundaryMap)
id = char(alignedRef.Reference.ID);
if ~isKey(boundaryMap, id)
    error("selectRRHDRoadPoints:BoundaryNotFound", "Boundary not found: %s", id);
end

geom = boundaryMap(id);
if string(alignedRef.Alignment) == "Backward"
    geom = flipud(geom);
end
end

function tf = routeMeetsQuality(route, distanceMeters, minEndpointSeparation, mapBounds, mapEdgeMargin)
if isempty(route) || size(route.points, 1) < 2
    tf = false;
    return
end

endpointSeparation = norm(route.pointB(1:2) - route.pointA(1:2));
plottedLength = polylineLength(route.points);
pointAInside = route.pointA(1) >= mapBounds(1, 1) + mapEdgeMargin ...
    && route.pointA(1) <= mapBounds(1, 2) - mapEdgeMargin ...
    && route.pointA(2) >= mapBounds(2, 1) + mapEdgeMargin ...
    && route.pointA(2) <= mapBounds(2, 2) - mapEdgeMargin;

tf = pointAInside ...
    && endpointSeparation >= minEndpointSeparation ...
    && abs(route.distance - distanceMeters) < 1e-6 ...
    && abs(plottedLength - distanceMeters) <= 20;
end

function bounds = drivingLaneBounds(lanes)
pts = zeros(0, 3);
for i = 1:numel(lanes)
    if string(lanes(i).LaneType) == "Driving" && size(lanes(i).Geometry, 1) >= 1
        pts = [pts; lanes(i).Geometry(:, 1:3)]; %#ok<AGROW>
    end
end

bounds = [min(pts(:, 1)), max(pts(:, 1)); min(pts(:, 2)), max(pts(:, 2))];
end

function [success, route, diagnostics] = findRandomQualityRoute( ...
    lanes, laneMap, boundaryMap, junctionLaneIds, candidateIdx, candidateDir, ...
    distanceMeters, minEndpointSeparation, mapBounds, fast, mapEdgeMargin, ...
    startStationRandomWindow, startStationAttempts)

diagnostics = struct;
diagnostics.RawRouteCount = 0;
diagnostics.BestEndpointSeparation = -Inf;
diagnostics.BestPolylineError = Inf;

qualityRoutes = {};
candidateOrder = 1:numel(candidateIdx);
if fast
    candidateOrder = candidateOrder(randperm(numel(candidateOrder)));
end

for orderIdx = 1:numel(candidateOrder)
    i = candidateOrder(orderIdx);
    startIdx = candidateIdx(i);
    startDir = candidateDir(i);
    stationAttemptCount = 1;
    if fast
        stationAttemptCount = max(1, startStationAttempts);
    end

    for stationAttempt = 1:stationAttemptCount
        startS = chooseStartStation(lanes(startIdx), startDir, fast, startStationRandomWindow);

        [ok, candidateRoute] = walkFirstContinuousRoute( ...
            lanes, laneMap, boundaryMap, junctionLaneIds, ...
            startIdx, startDir, startS, distanceMeters);

        if ~ok
            continue
        end

        diagnostics.RawRouteCount = diagnostics.RawRouteCount + 1;
        endpointSeparation = norm(candidateRoute.pointB(1:2) - candidateRoute.pointA(1:2));
        polylineError = abs(polylineLength(candidateRoute.points) - distanceMeters);
        if endpointSeparation > diagnostics.BestEndpointSeparation
            diagnostics.BestEndpointSeparation = endpointSeparation;
            diagnostics.BestPolylineError = polylineError;
        end

        if routeMeetsQuality(candidateRoute, distanceMeters, minEndpointSeparation, mapBounds, mapEdgeMargin)
            if fast
                success = true;
                route = candidateRoute;
                return
            else
                qualityRoutes{end + 1} = candidateRoute; %#ok<AGROW>
            end
        end
    end
end

success = ~isempty(qualityRoutes);
if success
    route = qualityRoutes{randi(numel(qualityRoutes))};
else
    route = [];
end
end

function startS = chooseStartStation(lane, dirSign, fast, startStationRandomWindow)
defaultStartS = 5;
if ~fast
    startS = defaultStartS;
    return
end

try
    geom = travelLanePolyline(lane, dirSign);
    len = polylineLength(geom);
catch
    startS = defaultStartS;
    return
end

if len <= 10
    startS = min(defaultStartS, len);
else
    randomWindow = min(max(0, startStationRandomWindow), len - 10);
    startS = defaultStartS + rand() * randomWindow;
end
end

function [success, route] = walkFirstContinuousRoute( ...
    lanes, laneMap, boundaryMap, junctionLaneIds, startIdx, startDir, startS, distanceMeters)

remain = distanceMeters;
idx = startIdx;
dirSign = startDir;
localS = startS;
routePoints = zeros(0, 3);
routeLaneIds = strings(0, 1);
traveled = 0;
maxHops = 100;
success = false;
route = [];

for hop = 1:maxHops
    lane = lanes(idx);
    geom = travelLanePolyline(lane, dirSign);
    len = polylineLength(geom);
    localS = min(localS, len);
    available = len - localS;

    if remain <= available + 1e-9
        endS = localS + remain;
        if isJunctionLane(lane, junctionLaneIds)
            return
        end

        routePoints = appendSegment(routePoints, polylineSegment(geom, localS, endS));
        traveled = traveled + (endS - localS);
        routeLaneIds(end + 1, 1) = string(lane.ID); %#ok<AGROW>

        route = struct;
        route.pointA = interpolatePolylineAtDistance( ...
            leftSideRoadPolyline(lanes(startIdx), startDir, boundaryMap), startS);
        route.pointB = interpolatePolylineAtDistance( ...
            leftSideRoadPolyline(lane, dirSign, boundaryMap), endS);
        route.points = routePoints;
        route.startIdx = startIdx;
        route.endIdx = idx;
        route.startDir = startDir;
        route.endDir = dirSign;
        route.laneIds = routeLaneIds;
        route.distance = traveled;
        success = abs(traveled - distanceMeters) < 1e-3;
        return
    end

    routePoints = appendSegment(routePoints, polylineSegment(geom, localS, len));
    traveled = traveled + available;
    remain = remain - available;
    routeLaneIds(end + 1, 1) = string(lane.ID); %#ok<AGROW>

    [nextIdx, nextDir] = chooseNearestContinuousLane( ...
        lanes, laneMap, idx, dirSign, geom(end, :));
    if nextIdx == 0
        return
    end

    idx = nextIdx;
    dirSign = nextDir;
    localS = 0;
end
end

function [nextIdx, nextDir] = chooseNearestContinuousLane(lanes, laneMap, idx, dirSign, currentEndPoint)
if dirSign == 1
    refs = lanes(idx).Successors;
else
    refs = lanes(idx).Predecessors;
end

nextIdx = 0;
nextDir = 0;
bestGap = Inf;

for i = 1:numel(refs)
    id = char(refs(i).Reference.ID);
    if ~isKey(laneMap, id)
        continue
    end

    candidateIdx = laneMap(id);
    candidateLane = lanes(candidateIdx);
    if string(candidateLane.LaneType) ~= "Driving"
        continue
    end

    candidateDir = 1;
    if string(refs(i).Alignment) == "Backward"
        candidateDir = -1;
    end

    try
        candidateGeom = travelLanePolyline(candidateLane, candidateDir);
    catch
        continue
    end
    gap = norm(candidateGeom(1, :) - currentEndPoint);
    if gap < bestGap
        bestGap = gap;
        nextIdx = candidateIdx;
        nextDir = candidateDir;
    end
end

if bestGap > 5
    nextIdx = 0;
    nextDir = 0;
end
end

function dist = cumulativeDistance(geom)
if size(geom, 1) < 2
    dist = 0;
    return
end

deltas = diff(geom(:, 1:3), 1, 1);
dist = [0; cumsum(sqrt(sum(deltas.^2, 2)))];
end

function len = polylineLength(geom)
dist = cumulativeDistance(geom);
len = dist(end);
end

function pts = interpolatePolylineAtFractions(geom, fractions)
len = polylineLength(geom);
if len <= eps
    pts = repmat(geom(1, :), numel(fractions), 1);
    return
end
pts = zeros(numel(fractions), 3);
for i = 1:numel(fractions)
    pts(i, :) = interpolatePolylineAtDistance(geom, fractions(i) * len);
end
end

function pt = interpolatePolylineAtDistance(geom, queryDistance)
dist = cumulativeDistance(geom);
queryDistance = max(0, min(queryDistance, dist(end)));

if queryDistance <= 0
    pt = geom(1, :);
    return
elseif queryDistance >= dist(end)
    pt = geom(end, :);
    return
end

segIdx = find(dist <= queryDistance, 1, "last");
if segIdx >= numel(dist)
    pt = geom(end, :);
    return
end

segLen = dist(segIdx + 1) - dist(segIdx);
if segLen <= eps
    pt = geom(segIdx, :);
else
    t = (queryDistance - dist(segIdx)) / segLen;
    pt = geom(segIdx, :) + t .* (geom(segIdx + 1, :) - geom(segIdx, :));
end
end

function segment = polylineSegment(geom, startS, endS)
dist = cumulativeDistance(geom);
startS = max(0, min(startS, dist(end)));
endS = max(0, min(endS, dist(end)));
if endS < startS
    tmp = startS;
    startS = endS;
    endS = tmp;
end

startPt = interpolatePolylineAtDistance(geom, startS);
endPt = interpolatePolylineAtDistance(geom, endS);
inside = dist > startS & dist < endS;
segment = [startPt; geom(inside, :); endPt];
end

function pts = appendSegment(pts, segment)
if isempty(segment)
    return
end

if isempty(pts)
    pts = segment;
elseif norm(pts(end, :) - segment(1, :)) < 1e-9
    pts = [pts; segment(2:end, :)];
else
    pts = [pts; segment];
end
end

function plotSelection(rrMap, result, plotFile)
fig = figure("Name", "RRHD Random 500 m Road Points", "Color", "w");
ax = axes(fig);
hold(ax, "on");
axis(ax, "equal");
grid(ax, "on");
box(ax, "on");
title(ax, "RRHD random left-side road point and 500 m drivable route");
xlabel(ax, "X (m)");
ylabel(ax, "Y (m)");

plotLaneSet(ax, rrMap.LaneBoundaries, [0.80 0.80 0.80], 0.25);
drivingLanes = rrMap.Lanes([rrMap.Lanes.LaneType] == "Driving");
plotLaneSet(ax, drivingLanes, [0.45 0.45 0.45], 0.35);

routePts = result.RoutePolyline;
plot(ax, routePts(:, 1), routePts(:, 2), "-", ...
    "Color", [0.85 0.05 0.55], "LineWidth", 2.0);
plot(ax, result.PointA(1), result.PointA(2), "o", ...
    "MarkerFaceColor", [0.10 0.45 0.95], "MarkerEdgeColor", "k", "MarkerSize", 8);
plot(ax, result.PointB(1), result.PointB(2), "s", ...
    "MarkerFaceColor", [0.95 0.20 0.10], "MarkerEdgeColor", "k", "MarkerSize", 8);
text(ax, result.PointA(1), result.PointA(2), "  A", "FontWeight", "bold", "Color", [0.10 0.20 0.70]);
text(ax, result.PointB(1), result.PointB(2), "  B", "FontWeight", "bold", "Color", [0.70 0.10 0.05]);
legend(ax, ["Lane boundaries", "Driving lanes", "500 m route", "Point A", "Point B"], ...
    "Location", "bestoutside");

pad = 25;
xmin = min(routePts(:, 1)) - pad;
xmax = max(routePts(:, 1)) + pad;
ymin = min(routePts(:, 2)) - pad;
ymax = max(routePts(:, 2)) + pad;
xlim(ax, [xmin, xmax]);
ylim(ax, [ymin, ymax]);

exportgraphics(fig, plotFile, "Resolution", 200);
close(fig);
end

function plotLaneSet(ax, items, color, lineWidth)
for i = 1:numel(items)
    if ~isprop(items(i), "Geometry")
        continue
    end
    geom = items(i).Geometry;
    if isnumeric(geom) && size(geom, 1) >= 2
        plot(ax, geom(:, 1), geom(:, 2), "-", "Color", color, "LineWidth", lineWidth);
    end
end
end
