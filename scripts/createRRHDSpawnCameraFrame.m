function outputs = createRRHDSpawnCameraFrame()
% createRRHDSpawnCameraFrame Configure a verified RRHD spawn and save a camera frame.
%
% The spawn point is selected from a non-junction driving lane in the
% IndianCityBlock RRHD map. It is offset to the left side of the lane travel
% direction and has been visually verified in the Unreal scene used by
% ControlVehicle.

repoRoot = fileparts(fileparts(mfilename("fullpath")));
rrhdFile = fullfile(repoRoot, "localFolder", "IndianCityBlock.rrhd");
outDir = fullfile(repoRoot, "analysis_outputs");
if ~isfolder(outDir)
    mkdir(outDir);
end

if ~isfile(rrhdFile)
    error("createRRHDSpawnCameraFrame:FileNotFound", ...
        "RRHD file not found: %s", rrhdFile);
end

% This point is in RRHD/RoadRunner XY coordinates for this scene. It is 0.875 m
% left of the nearest non-junction driving lane centerline.
rrhdSpawnPoint = [-24.031464, 16.369602, 0.170000];
rrhdSpawnYawDeg = 89.634893;
stopTime = "0.5";

rrMap = roadrunnerHDMap;
read(rrMap, rrhdFile);

laneInfo = findNearestDrivingLane(rrMap, rrhdSpawnPoint);
assertSpawnIsUsable(laneInfo, rrhdSpawnPoint, rrhdSpawnYawDeg);

[initialPos, initialRot, transformDetails] = convertRRHDPoseToSim3DInitialPose( ...
    rrhdSpawnPoint, rrhdSpawnYawDeg);

plotFile = fullfile(outDir, "IndianCityBlock_rrhd_spawn_point.png");
plotRRHDSpawn(rrMap, laneInfo, rrhdSpawnPoint, rrhdSpawnYawDeg, plotFile);

modelName = "ControlVehicle";
cameraVideoFile = fullfile(outDir, "spawn_camera_check.avi");
cameraFrameFile = fullfile(outDir, "spawn_camera_check_frame.png");

configureControlVehicle(modelName, initialPos, initialRot, stopTime, cameraVideoFile);
captureCameraFrame(modelName, stopTime, cameraVideoFile, cameraFrameFile);

outputs = struct;
outputs.RRHDFile = string(rrhdFile);
outputs.SpawnPoint = rrhdSpawnPoint;
outputs.SpawnYawDeg = rrhdSpawnYawDeg;
outputs.RRHDSpawnPoint = rrhdSpawnPoint;
outputs.RRHDSpawnYawDeg = rrhdSpawnYawDeg;
outputs.InitialPos = initialPos;
outputs.InitialRot = initialRot;
outputs.TransformDetails = transformDetails;
outputs.NearestLaneID = string(laneInfo.Lane.ID);
outputs.DistanceFromLaneCenter = laneInfo.Distance;
outputs.LeftOffsetFromLaneCenter = laneInfo.LeftOffset;
outputs.SpawnPlot = string(plotFile);
outputs.CameraVideo = string(cameraVideoFile);
outputs.CameraFrame = string(cameraFrameFile);

fprintf("Configured ControlVehicle spawn point\n");
fprintf("  RRHD point:  [%.6f, %.6f, %.6f]\n", rrhdSpawnPoint);
fprintf("  RRHD yaw:    %.6f deg\n", rrhdSpawnYawDeg);
fprintf("  InitialPos:  [%.6f, %.6f, %.6f]\n", initialPos);
fprintf("  InitialRot:  [%.6f, %.6f, %.6f] rad\n", initialRot);
fprintf("  Nearest non-junction lane: %s\n", outputs.NearestLaneID);
fprintf("  Lane-center distance: %.3f m; left offset: %.3f m\n", ...
    outputs.DistanceFromLaneCenter, outputs.LeftOffsetFromLaneCenter);
fprintf("  RRHD spawn plot: %s\n", plotFile);
fprintf("  Camera frame:    %s\n", cameraFrameFile);
end

function laneInfo = findNearestDrivingLane(rrMap, point)
lanes = rrMap.Lanes;
junctionLaneIDs = collectJunctionLaneIDs(rrMap.Junctions);

laneInfo = struct( ...
    "Lane", [], ...
    "LaneIndex", 0, ...
    "Distance", Inf, ...
    "Station", NaN, ...
    "Projection", [NaN, NaN, NaN], ...
    "SegmentIndex", 1, ...
    "IsJunction", false);

for i = 1:numel(lanes)
    lane = lanes(i);
    if string(lane.LaneType) ~= "Driving" || size(lane.Geometry, 1) < 2
        continue
    end

    [distance, station, projection, segmentIndex] = pointPolylineDistance(point, lane.Geometry);
    if distance < laneInfo.Distance
        laneInfo.Lane = lane;
        laneInfo.LaneIndex = i;
        laneInfo.Distance = distance;
        laneInfo.Station = station;
        laneInfo.Projection = projection;
        laneInfo.SegmentIndex = segmentIndex;
        laneInfo.IsJunction = any(junctionLaneIDs == string(lane.ID));
    end
end

if laneInfo.LaneIndex == 0
    error("createRRHDSpawnCameraFrame:NoDrivingLane", ...
        "No driving lane was found near the spawn point.");
end

laneInfo.LaneYawDeg = laneTravelYaw(laneInfo.Lane, laneInfo.SegmentIndex);
leftNormal = [-sind(laneInfo.LaneYawDeg), cosd(laneInfo.LaneYawDeg), 0];
laneInfo.LeftOffset = dot(point - laneInfo.Projection, leftNormal);
end

function ids = collectJunctionLaneIDs(junctions)
ids = strings(0, 1);
for i = 1:numel(junctions)
    refs = junctions(i).Lanes;
    for j = 1:numel(refs)
        ids(end + 1, 1) = string(refs(j).ID); %#ok<AGROW>
    end
end
ids = unique(ids);
end

function [distance, station, projection, segmentIndex] = pointPolylineDistance(point, geom)
distance = Inf;
station = 0;
projection = geom(1, 1:3);
segmentIndex = 1;
accumulated = 0;

for i = 1:size(geom, 1) - 1
    a = geom(i, 1:3);
    b = geom(i + 1, 1:3);
    ab = b - a;
    if dot(ab, ab) <= eps
        t = 0;
    else
        t = max(0, min(1, dot(point - a, ab) / dot(ab, ab)));
    end

    p = a + t .* ab;
    d = norm(point(1:2) - p(1:2));
    if d < distance
        distance = d;
        station = accumulated + norm(p - a);
        projection = p;
        segmentIndex = i;
    end

    accumulated = accumulated + norm(ab);
end
end

function yawDeg = laneTravelYaw(lane, segmentIndex)
geom = lane.Geometry;
segmentIndex = max(1, min(segmentIndex, size(geom, 1) - 1));
delta = geom(segmentIndex + 1, 1:2) - geom(segmentIndex, 1:2);
yawDeg = atan2d(delta(2), delta(1));
if string(lane.TravelDirection) == "Backward"
    yawDeg = wrapTo180(yawDeg + 180);
end
end

function assertSpawnIsUsable(laneInfo, point, spawnYawDeg)
if laneInfo.IsJunction
    error("createRRHDSpawnCameraFrame:JunctionSpawn", ...
        "Spawn point is closest to a junction lane: %s", string(laneInfo.Lane.ID));
end

if laneInfo.Distance > 2.0
    error("createRRHDSpawnCameraFrame:SpawnOffLane", ...
        "Spawn point is %.3f m from the nearest driving lane centerline.", ...
        laneInfo.Distance);
end

if laneInfo.LeftOffset <= 0
    error("createRRHDSpawnCameraFrame:SpawnNotLeftSide", ...
        "Spawn point is not on the left side of the lane travel direction.");
end

yawError = abs(wrapTo180(spawnYawDeg - laneInfo.LaneYawDeg));
if yawError > 5
    warning("createRRHDSpawnCameraFrame:YawMismatch", ...
        "Spawn yaw %.3f deg differs from nearest lane yaw %.3f deg by %.3f deg.", ...
        spawnYawDeg, laneInfo.LaneYawDeg, yawError);
end

fprintf("Spawn validation: lane distance %.3f m, left offset %.3f m, lane yaw %.3f deg.\n", ...
    laneInfo.Distance, laneInfo.LeftOffset, laneInfo.LaneYawDeg);
fprintf("RRHD spawn point: [%.6f, %.6f, %.6f]\n", point);
end

function plotRRHDSpawn(rrMap, laneInfo, spawnPoint, spawnYawDeg, plotFile)
fig = figure("Name", "RRHD Spawn Point", "Color", "w");
ax = axes(fig);
hold(ax, "on");
axis(ax, "equal");
grid(ax, "on");
box(ax, "on");
set(ax, ...
    "Color", "w", ...
    "XColor", [0.10 0.10 0.10], ...
    "YColor", [0.10 0.10 0.10], ...
    "GridColor", [0.82 0.82 0.82], ...
    "MinorGridColor", [0.90 0.90 0.90]);
title(ax, "RRHD spawn point for ControlVehicle", "Color", [0.10 0.10 0.10]);
xlabel(ax, "X (m)", "Color", [0.10 0.10 0.10]);
ylabel(ax, "Y (m)", "Color", [0.10 0.10 0.10]);

hBoundaries = plotLaneSet(ax, rrMap.LaneBoundaries, [0.70 0.70 0.70], 0.25);
drivingLanes = rrMap.Lanes([rrMap.Lanes.LaneType] == "Driving");
hDriving = plotLaneSet(ax, drivingLanes, [0.35 0.35 0.35], 0.35);

selectedGeom = laneInfo.Lane.Geometry;
hSelected = plot(ax, selectedGeom(:, 1), selectedGeom(:, 2), "-", ...
    "Color", [0.05 0.45 0.90], "LineWidth", 2.0);
hSpawn = plot(ax, spawnPoint(1), spawnPoint(2), "o", ...
    "MarkerFaceColor", [0.95 0.20 0.10], ...
    "MarkerEdgeColor", "k", "MarkerSize", 9);

arrowLength = 12;
hHeading = quiver(ax, spawnPoint(1), spawnPoint(2), ...
    arrowLength * cosd(spawnYawDeg), arrowLength * sind(spawnYawDeg), ...
    0, "Color", [0.95 0.20 0.10], "LineWidth", 1.8, "MaxHeadSize", 0.8);
text(ax, spawnPoint(1), spawnPoint(2), "  Spawn", ...
    "FontWeight", "bold", "Color", [0.70 0.10 0.05]);

lgd = legend(ax, [hBoundaries, hDriving, hSelected, hSpawn, hHeading], ...
    ["Lane boundaries", "Driving lanes", "Selected lane", "Spawn point", "Spawn heading"], ...
    "Location", "bestoutside");
set(lgd, "TextColor", [0.10 0.10 0.10], "Color", "w");

viewRadius = 65;
xlim(ax, [spawnPoint(1) - viewRadius, spawnPoint(1) + viewRadius]);
ylim(ax, [spawnPoint(2) - viewRadius, spawnPoint(2) + viewRadius]);

exportgraphics(fig, plotFile, "Resolution", 200);
close(fig);
end

function firstHandle = plotLaneSet(ax, items, color, lineWidth)
firstHandle = gobjects(1);
for i = 1:numel(items)
    if ~isprop(items(i), "Geometry")
        continue
    end
    geom = items(i).Geometry;
    if isnumeric(geom) && size(geom, 1) >= 2
        h = plot(ax, geom(:, 1), geom(:, 2), "-", "Color", color, "LineWidth", lineWidth);
        if isgraphics(firstHandle)
            set(h, "HandleVisibility", "off");
        else
            firstHandle = h;
        end
    end
end
end

function configureControlVehicle(modelName, initialPos, initialRot, stopTime, cameraVideoFile)
open_system(modelName);

vehicleBlock = modelName + "/Simulation 3D Physics Vehicle";
set_param(vehicleBlock, "InitialPos", mat2str(initialPos, 8));
set_param(vehicleBlock, "InitialRot", mat2str(initialRot, 8));
set_param(modelName, "StopTime", stopTime);

cameraFileBlock = modelName + "/Sensors/CameraFile";
if ~ishandle(getSimulinkBlockHandle(cameraFileBlock))
    add_block("dspvision/To Multimedia File", cameraFileBlock, ...
        "Position", [1335 95 1455 145]);
end

set_param(cameraFileBlock, ...
    "outputFilename", relativePathForModel(cameraVideoFile), ...
    "streamSelection", "Video only", ...
    "imagePorts", "One multidimensional signal");

ports = get_param(cameraFileBlock, "PortHandles");
if get_param(ports.Inport(1), "Line") == -1
    add_line(modelName + "/Sensors", "Simulation 3D Camera/1", "CameraFile/1", ...
        "autorouting", "on");
end

save_system(modelName);
end

function captureCameraFrame(modelName, stopTime, cameraVideoFile, cameraFrameFile)
clear("vr");
evalin("base", "clear vr frame");
if isfile(cameraVideoFile)
    try
        delete(cameraVideoFile);
    catch ME
        warning("createRRHDSpawnCameraFrame:VideoDeleteFailed", ...
            "Could not delete existing camera video before simulation: %s", ME.message);
    end
end
if isfile(cameraFrameFile)
    delete(cameraFrameFile);
end

sim(modelName, "StopTime", stopTime);

vr = VideoReader(cameraVideoFile);
frame = readFrame(vr);
imwrite(frame, cameraFrameFile);
clear("vr", "frame");
end

function relPath = relativePathForModel(absPath)
repoRoot = fileparts(fileparts(mfilename("fullpath")));
try
    relPath = char(erase(string(absPath), string(repoRoot) + filesep));
catch
    relPath = char(absPath);
end
end
