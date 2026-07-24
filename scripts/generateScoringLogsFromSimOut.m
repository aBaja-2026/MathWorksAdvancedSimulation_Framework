function [telemetry, events, score] = generateScoringLogsFromSimOut(simOut, options)
% generateScoringLogsFromSimOut Export scoring logs from ControlVehicle sim output.
%
% The model must log Vehicle_Info as Vehicle_Info_Log in logsout. This
% function converts Simulation 3D vehicle signals into RRHD/map-frame
% telemetry, computes lane deviation from the RRHD map, writes the CSV/JSON
% logs, and returns the scoreNavigationRun result.

arguments
    simOut
    options.OutputFolder (1, 1) string = fullfile(pwd, "results", "logs")
    options.TelemetryFile (1, 1) string = "telemetry_log.csv"
    options.EventsFile (1, 1) string = "events_log.json"
    options.ScoreFile (1, 1) string = "score_summary.json"
    options.RRHDFile (1, 1) string = fullfile(pwd, "localFolder", "IndianCityBlock.rrhd")
    options.PointB (1, 3) double = [NaN, NaN, NaN]
    options.PointBRadius_m (1, 1) double = 5
    options.SampleTime_s (1, 1) double = 0.1
    options.VehicleLength_m (1, 1) double = 2.1
    options.VehicleWidth_m (1, 1) double = 1.4
    options.EgoOriginRef (1, 1) string = "rear_axle_center"
    options.IndicatorState (1, 1) string {mustBeMember(options.IndicatorState, ["none", "left", "right"])} = "none"
    options.GenerateLaneChangeEvents (1, 1) logical = false
    options.TMin_s (1, 1) double = NaN
    options.TMax_s (1, 1) double = NaN
    options.YellowLineViolationCount (1, 1) double = NaN
    options.MinimumObstacleDistance_m (1, 1) double = NaN
end

logsout = getLogsout(simOut);
vehicleInfo = logsout.get("Vehicle_Info_Log").Values;

raw = extractVehicleRaw(vehicleInfo);
sampleTime_s = options.SampleTime_s;
if sampleTime_s <= 0
    error("generateScoringLogsFromSimOut:InvalidSampleTime", ...
        "SampleTime_s must be positive.");
end

time_s = makeSampleTimes(raw.Time_s, sampleTime_s);
simX = interp1(raw.Time_s, raw.X, time_s, "linear", "extrap");
simY = interp1(raw.Time_s, raw.Y, time_s, "linear", "extrap");
simZ = interp1(raw.Time_s, raw.Z, time_s, "linear", "extrap");
simXdot = interp1(raw.Time_s, raw.Xdot, time_s, "linear", "extrap");
simYdot = interp1(raw.Time_s, raw.Ydot, time_s, "linear", "extrap");
simPsi = interp1(raw.Time_s, raw.Psi, time_s, "linear", "extrap");

ego_x = simX;
ego_y = -simY;
ego_speed_kmh = 3.6 .* hypot(simXdot, simYdot);
ego_heading_deg = headingFromVelocityOrYaw(simXdot, simYdot, simPsi);

vehiclePositionSim3D = [simX, simY, simZ];
[lateral_deviation_m, laneDetails] = computeLaneDeviationFromRRHD( ...
    vehiclePositionSim3D, options.RRHDFile, CoordinateFrame="Sim3D");

task_complete = computeTaskComplete( ...
    [ego_x, ego_y, simZ], ego_speed_kmh, resolvePointB(options.PointB), options.PointBRadius_m);

n = numel(time_s);
telemetry = table( ...
    time_s(:), ...
    ego_x(:), ...
    ego_y(:), ...
    ego_speed_kmh(:), ...
    ego_heading_deg(:), ...
    lateral_deviation_m(:), ...
    repmat(options.IndicatorState, n, 1), ...
    repmat(options.VehicleLength_m, n, 1), ...
    repmat(options.VehicleWidth_m, n, 1), ...
    repmat(options.EgoOriginRef, n, 1), ...
    zeros(n, 1), ...
    task_complete(:), ...
    'VariableNames', [ ...
        "timestamp_s", ...
        "ego_x", ...
        "ego_y", ...
        "ego_speed_kmh", ...
        "ego_heading_deg", ...
        "lateral_deviation_m", ...
        "indicator_state", ...
        "vehicle_length_m", ...
        "vehicle_width_m", ...
        "ego_origin_ref", ...
        "collision_flag", ...
        "task_complete"]);

events = buildEvents(telemetry, laneDetails, options.GenerateLaneChangeEvents);

if ~isfolder(options.OutputFolder)
    mkdir(options.OutputFolder);
end

telemetryPath = fullfile(options.OutputFolder, options.TelemetryFile);
eventsPath = fullfile(options.OutputFolder, options.EventsFile);
scorePath = fullfile(options.OutputFolder, options.ScoreFile);

writetable(telemetry, telemetryPath);
writeEventsJson(eventsPath, events);

score = scoreNavigationRun(telemetry, events, ...
    TMin_s=options.TMin_s, ...
    TMax_s=options.TMax_s, ...
    YellowLineViolationCount=options.YellowLineViolationCount, ...
    MinimumObstacleDistance_m=options.MinimumObstacleDistance_m, ...
    IgnoreSignalStates=true);
score.OutputFiles = struct( ...
    "Telemetry", string(telemetryPath), ...
    "Events", string(eventsPath), ...
    "Score", string(scorePath));
writeTextFile(scorePath, jsonencode(score, PrettyPrint=true));
end

function logsout = getLogsout(simOut)
if isa(simOut, "Simulink.SimulationOutput")
    logsout = simOut.logsout;
elseif isa(simOut, "Simulink.SimulationData.Dataset")
    logsout = simOut;
else
    error("generateScoringLogsFromSimOut:InvalidInput", ...
        "simOut must be a Simulink.SimulationOutput or Simulink.SimulationData.Dataset.");
end

if isempty(logsout.get("Vehicle_Info_Log"))
    error("generateScoringLogsFromSimOut:MissingVehicleInfoLog", ...
        "logsout must contain a signal named Vehicle_Info_Log.");
end
end

function raw = extractVehicleRaw(vehicleInfo)
x = vehicleInfo.InertFrm.Cg.Disp.X;
y = vehicleInfo.InertFrm.Cg.Disp.Y;
z = vehicleInfo.InertFrm.Cg.Disp.Z;
xdot = vehicleInfo.InertFrm.Cg.Vel.Xdot;
ydot = vehicleInfo.InertFrm.Cg.Vel.Ydot;
zdot = vehicleInfo.InertFrm.Cg.Vel.Zdot;
psi = vehicleInfo.InertFrm.Cg.Ang.psi;

raw = struct;
raw.Time_s = x.Time(:);
raw.X = tsData(x);
raw.Y = tsData(y);
raw.Z = tsData(z);
raw.Xdot = tsData(xdot);
raw.Ydot = tsData(ydot);
raw.Zdot = tsData(zdot);
raw.Psi = unwrap(tsData(psi));
end

function data = tsData(ts)
data = squeeze(ts.Data);
data = data(:);
end

function time_s = makeSampleTimes(rawTime_s, sampleTime_s)
startTime = rawTime_s(1);
stopTime = rawTime_s(end);
time_s = (startTime:sampleTime_s:stopTime).';
if isempty(time_s) || time_s(end) < stopTime
    time_s(end + 1, 1) = stopTime;
end
end

function heading_deg = headingFromVelocityOrYaw(simXdot, simYdot, simPsi)
speed_mps = hypot(simXdot, simYdot);
headingFromVelocity = wrapTo360Local(atan2d(-simYdot, simXdot));
headingFromYaw = wrapTo360Local(270 - rad2deg(simPsi));
heading_deg = headingFromVelocity;
heading_deg(speed_mps < 0.05) = headingFromYaw(speed_mps < 0.05);
end

function angle_deg = wrapTo360Local(angle_deg)
angle_deg = mod(angle_deg, 360);
angle_deg(angle_deg < 0) = angle_deg(angle_deg < 0) + 360;
end

function pointB = resolvePointB(optionPointB)
pointB = optionPointB;
if any(~isfinite(pointB))
    try
        pointB = evalin("base", "carEndPoint");
    catch
        pointB = [NaN, NaN, NaN];
    end
end
end

function taskComplete = computeTaskComplete(positionMap, speed_kmh, pointB, radius_m)
taskComplete = zeros(size(speed_kmh));
if any(~isfinite(pointB)) || radius_m <= 0
    return
end

distanceToPointB = hypot(positionMap(:, 1) - pointB(1), positionMap(:, 2) - pointB(2));
taskComplete(distanceToPointB <= radius_m & speed_kmh < 0.5) = 1;
end

function events = buildEvents(telemetry, laneDetails, generateLaneChangeEvents)
if ~generateLaneChangeEvents
    events = emptyEvents();
    return
end

laneId = laneDetails.LaneID(:);
if numel(laneId) < 2
    events = emptyEvents();
    return
end

laneChangeIdx = find(laneId(2:end) ~= laneId(1:end - 1)) + 1;
events = repmat(emptyEvent(), 1, 2 * numel(laneChangeIdx));
eventIdx = 0;

for i = 2:numel(laneId)
    if laneId(i) == laneId(i - 1)
        continue
    end

    eventIdx = eventIdx + 1;
    events(eventIdx) = makeEvent( ...
        telemetry.timestamp_s(i), ...
        "lane_change_start", ...
        telemetry.ego_x(i), ...
        telemetry.ego_y(i), ...
        struct( ...
            "from_lane", laneId(i - 1), ...
            "to_lane", laneId(i), ...
            "indicator_active", telemetry.indicator_state(i) ~= "none"));

    eventIdx = eventIdx + 1;
    events(eventIdx) = makeEvent( ...
        telemetry.timestamp_s(i), ...
        "lane_change_end", ...
        telemetry.ego_x(i), ...
        telemetry.ego_y(i), ...
        struct( ...
            "from_lane", laneId(i - 1), ...
            "to_lane", laneId(i), ...
            "success", true));
end
events = events(1:eventIdx);
end

function events = emptyEvents()
events = repmat(emptyEvent(), 1, 0);
end

function event = emptyEvent()
event = struct("timestamp_s", [], "event_type", "", "ego_x", [], "ego_y", [], "details", struct);
end

function event = makeEvent(timestamp_s, eventType, ego_x, ego_y, details)
event = struct( ...
    "timestamp_s", timestamp_s, ...
    "event_type", eventType, ...
    "ego_x", ego_x, ...
    "ego_y", ego_y, ...
    "details", details);
end

function writeEventsJson(filePath, events)
if isempty(events)
    writeTextFile(filePath, "[]");
else
    writeTextFile(filePath, jsonencode(events, PrettyPrint=true));
end
end

function writeTextFile(filePath, text)
fid = fopen(filePath, "w");
if fid == -1
    error("generateScoringLogsFromSimOut:FileOpenFailed", ...
        "Could not open file for writing: %s", filePath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", text);
end
