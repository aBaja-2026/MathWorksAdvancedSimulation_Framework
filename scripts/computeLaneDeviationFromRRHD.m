function [lateralDeviation_m, details] = computeLaneDeviationFromRRHD(vehiclePosition, rrhdSource, options)
% computeLaneDeviationFromRRHD Compute signed lane-center deviation from RRHD lanes.
%
% lateralDeviation_m is positive to the right of the lane travel direction,
% matching the aBAJA telemetry_log.csv convention. By default, vehiclePosition
% is interpreted in Simulation 3D coordinates and converted to RRHD/map
% coordinates as [x, y, z] -> [x, -y, z].

arguments
    vehiclePosition (:, :) double
    rrhdSource = []
    options.CoordinateFrame (1, 1) string {mustBeMember(options.CoordinateFrame, ["Sim3D", "RRHD"])} = "Sim3D"
    options.LaneType (1, 1) string = "Driving"
    options.ExcludeJunctionLanes (1, 1) logical = false
end

validateattributes(vehiclePosition, {'numeric'}, {'real', '2d'}, ...
    mfilename, 'vehiclePosition', 1);
if ~ismember(size(vehiclePosition, 2), [2, 3])
    error("computeLaneDeviationFromRRHD:InvalidVehiclePosition", ...
        "vehiclePosition must have 2 or 3 columns.");
end

mapPosition = toMapPosition(vehiclePosition, options.CoordinateFrame);
[lanes, junctionLaneIds, sourceFile] = readLaneSource(rrhdSource);

if isempty(lanes)
    error("computeLaneDeviationFromRRHD:NoLanes", ...
        "No lanes were found in the RRHD source.");
end

n = size(mapPosition, 1);
lateralDeviation_m = NaN(n, 1);
laneId = strings(n, 1);
laneIndex = zeros(n, 1);
station_m = NaN(n, 1);
projection = NaN(n, 3);
unsignedDistance_m = NaN(n, 1);
segmentIndex = zeros(n, 1);
travelDirection_xy = NaN(n, 2);

for i = 1:n
    best = nearestLaneProjection( ...
        mapPosition(i, :), lanes, junctionLaneIds, options.LaneType, options.ExcludeJunctionLanes);

    lateralDeviation_m(i) = best.LateralDeviation_m;
    laneId(i) = best.LaneID;
    laneIndex(i) = best.LaneIndex;
    station_m(i) = best.Station_m;
    projection(i, :) = best.Projection;
    unsignedDistance_m(i) = best.UnsignedDistance_m;
    segmentIndex(i) = best.SegmentIndex;
    travelDirection_xy(i, :) = best.TravelDirection_xy;
end

details = struct;
details.SourceFile = string(sourceFile);
details.InputCoordinateFrame = options.CoordinateFrame;
details.VehiclePositionInput = vehiclePosition;
details.VehiclePositionMap = mapPosition;
details.LaneID = laneId;
details.LaneIndex = laneIndex;
details.Station_m = station_m;
details.Projection = projection;
details.UnsignedDistance_m = unsignedDistance_m;
details.SegmentIndex = segmentIndex;
details.TravelDirection_xy = travelDirection_xy;
details.SignConvention = "positive right of lane travel direction";
details.PositionConversion = "[sim_x, sim_y, sim_z] -> [sim_x, -sim_y, sim_z] when CoordinateFrame is Sim3D";
end

function mapPosition = toMapPosition(vehiclePosition, coordinateFrame)
if size(vehiclePosition, 2) == 2
    vehiclePosition = [vehiclePosition, zeros(size(vehiclePosition, 1), 1)];
end

mapPosition = vehiclePosition;
if coordinateFrame == "Sim3D"
    mapPosition(:, 2) = -mapPosition(:, 2);
end
end

function [lanes, junctionLaneIds, sourceFile] = readLaneSource(rrhdSource)
sourceFile = "";
if nargin < 1 || isempty(rrhdSource)
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    rrhdSource = fullfile(repoRoot, "localFolder", "IndianCityBlock.rrhd");
end

if isstring(rrhdSource) || ischar(rrhdSource)
    sourceFile = string(rrhdSource);
    if ~isfile(sourceFile)
        error("computeLaneDeviationFromRRHD:FileNotFound", ...
            "RRHD file not found: %s", sourceFile);
    end
    rrMap = roadrunnerHDMap;
    read(rrMap, char(sourceFile));
    lanes = rrMap.Lanes;
    junctionLaneIds = collectJunctionLaneIds(rrMap.Junctions);
elseif isobject(rrhdSource) || isstruct(rrhdSource)
    if hasMember(rrhdSource, "Lanes")
        lanes = rrhdSource.Lanes;
    else
        lanes = rrhdSource;
    end
    if hasMember(rrhdSource, "Junctions")
        junctionLaneIds = collectJunctionLaneIds(rrhdSource.Junctions);
    else
        junctionLaneIds = strings(0, 1);
    end
else
    error("computeLaneDeviationFromRRHD:InvalidSource", ...
        "rrhdSource must be empty, an RRHD file path, a roadrunnerHDMap, or a lane struct/object array.");
end
end

function ids = collectJunctionLaneIds(junctions)
ids = strings(0, 1);
for i = 1:numel(junctions)
    if ~hasMember(junctions(i), "Lanes")
        continue
    end
    refs = junctions(i).Lanes;
    for j = 1:numel(refs)
        try
            ids(end + 1, 1) = string(refs(j).ID); %#ok<AGROW>
        catch
        end
    end
end
ids = unique(ids);
end

function best = nearestLaneProjection(point, lanes, junctionLaneIds, laneType, excludeJunctionLanes)
best = emptyBest();

for i = 1:numel(lanes)
    lane = lanes(i);
    if ~isUsableLane(lane, laneType, junctionLaneIds, excludeJunctionLanes)
        continue
    end

    geom = double(lane.Geometry);
    if size(geom, 2) == 2
        geom = [geom, zeros(size(geom, 1), 1)];
    end
    if size(geom, 1) < 2
        continue
    end

    candidate = nearestPointOnLane(point, geom, laneTravelDirection(lane), i, string(lane.ID));
    if candidate.UnsignedDistance_m < best.UnsignedDistance_m
        best = candidate;
    end
end

if ~isfinite(best.UnsignedDistance_m)
    error("computeLaneDeviationFromRRHD:NoUsableLane", ...
        "No usable %s lane geometry was found.", laneType);
end
end

function tf = isUsableLane(lane, laneType, junctionLaneIds, excludeJunctionLanes)
tf = false;
if ~hasMember(lane, "Geometry") || isempty(lane.Geometry)
    return
end
if hasMember(lane, "LaneType") && string(lane.LaneType) ~= laneType
    return
end
if excludeJunctionLanes && any(junctionLaneIds == string(lane.ID))
    return
end
tf = true;
end

function directionSign = laneTravelDirection(lane)
directionSign = 1;
if hasMember(lane, "TravelDirection")
    if string(lane.TravelDirection) == "Backward"
        directionSign = -1;
    end
end
end

function tf = hasMember(value, name)
if isstruct(value)
    tf = isfield(value, name);
else
    tf = isprop(value, name);
end
end

function best = nearestPointOnLane(point, geom, directionSign, laneIndex, laneId)
best = emptyBest();
stationAtSegmentStart = 0;

for j = 1:size(geom, 1) - 1
    p0 = geom(j, 1:3);
    p1 = geom(j + 1, 1:3);
    segment = p1 - p0;
    segmentLength = norm(segment(1:2));
    if segmentLength <= eps
        continue
    end

    t = dot(point(1:2) - p0(1:2), segment(1:2)) / dot(segment(1:2), segment(1:2));
    t = max(0, min(1, t));
    projection = p0 + t .* segment;
    offset = point(1:2) - projection(1:2);
    unsignedDistance = norm(offset);

    travelDir = directionSign .* segment(1:2) ./ segmentLength;
    leftNormal = [-travelDir(2), travelDir(1)];
    lateralDeviation = -dot(offset, leftNormal);

    if unsignedDistance < best.UnsignedDistance_m
        best.LateralDeviation_m = lateralDeviation;
        best.LaneID = laneId;
        best.LaneIndex = laneIndex;
        best.Station_m = stationAtSegmentStart + t * segmentLength;
        best.Projection = projection;
        best.UnsignedDistance_m = unsignedDistance;
        best.SegmentIndex = j;
        best.TravelDirection_xy = travelDir;
    end

    stationAtSegmentStart = stationAtSegmentStart + segmentLength;
end
end

function best = emptyBest()
best = struct( ...
    "LateralDeviation_m", NaN, ...
    "LaneID", "", ...
    "LaneIndex", 0, ...
    "Station_m", NaN, ...
    "Projection", [NaN, NaN, NaN], ...
    "UnsignedDistance_m", Inf, ...
    "SegmentIndex", 0, ...
    "TravelDirection_xy", [NaN, NaN]);
end
