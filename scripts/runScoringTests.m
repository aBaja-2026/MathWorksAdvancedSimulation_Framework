function runScoringTests()
% runScoringTests Deterministic checks for scoreNavigationRun.

fprintf("Running scoring tests...\n");

testPerfectKnownRun();
testPenaltyRun();
testUnknownOfficialGeometryMetrics();
testSignalStatesIgnored();
testTelemetrySchemaValidation();

fprintf("Scoring tests passed.\n");
end

function testPerfectKnownRun()
telemetry = makeTelemetry(0:0.1:10);
telemetry.ego_speed_kmh(:) = 24;
telemetry.lateral_deviation_m(:) = 0.05;
telemetry.task_complete(end) = 1;
events = makeLaneChangeEvent(true);

result = scoreNavigationRun(telemetry, events, ...
    TMin_s=5, ...
    TMax_s=15, ...
    RedSignalViolationCount=0, ...
    YellowLineViolationCount=0, ...
    MinimumObstacleDistance_m=1.25);

assertClose(result.Scores.TaskCompletion, 20);
assertClose(result.Scores.NavigationTime, 7.5);
assertClose(result.Scores.SpeedLimitCompliance, 10);
assert(isnan(result.Scores.RedSignalCompliance));
assertClose(result.Scores.LaneMarkingCompliance, 10);
assertClose(result.Scores.IndicatorCompliance, 5);
assertClose(result.Scores.MinimumObstacleDistance, 5);
assertClose(result.Scores.LaneCenteringAccuracy, 5);
assertClose(result.Scores.CollisionPenalty, 0);
assertClose(result.TotalScore, 62.5);
assert(isequal(result.IgnoredMetrics, "RedSignalCompliance"));
assert(result.Validation.TelemetrySchemaValid);
assert(result.Validation.EventsSchemaValid);

    tmpTelemetry = [tempname, '.csv'];
    tmpEvents = [tempname, '.json'];
    cleanup = onCleanup(@() deleteIfExists({tmpTelemetry, tmpEvents}));
writetable(telemetry, tmpTelemetry);
writeText(tmpEvents, jsonencode(events));

fileResult = scoreNavigationRun(tmpTelemetry, tmpEvents, ...
    TMin_s=5, ...
    TMax_s=15, ...
    RedSignalViolationCount=0, ...
    YellowLineViolationCount=0, ...
    MinimumObstacleDistance_m=1.25);
assertClose(fileResult.TotalScore, result.TotalScore);
end

function testPenaltyRun()
telemetry = makeTelemetry(0:0.1:4);
telemetry.ego_speed_kmh(:) = 20;
telemetry.ego_speed_kmh(11:21) = 35;
telemetry.lateral_deviation_m(:) = 0.25;
telemetry.collision_flag(6:8) = 1;
telemetry.collision_flag(30:35) = 1;
telemetry.task_complete(:) = 0;
events = makeLaneChangeEvent(false);

result = scoreNavigationRun(telemetry, events, ...
    TMin_s=5, ...
    TMax_s=15, ...
    RedSignalViolationCount=2, ...
    YellowLineViolationCount=4, ...
    MinimumObstacleDistance_m=0.3);

assertClose(result.Scores.TaskCompletion, 0);
assertClose(result.Scores.NavigationTime, 0);
assertClose(result.Metrics.OverspeedDuration_s, 1.1);
assertClose(result.Scores.SpeedLimitCompliance, 8.9);
assert(result.Metrics.CollisionEventCount == 2);
assertClose(result.Scores.CollisionPenalty, -10);
assert(isnan(result.Scores.RedSignalCompliance));
assertClose(result.Scores.LaneMarkingCompliance, 0);
assertClose(result.Scores.IndicatorCompliance, 3);
assertClose(result.Scores.MinimumObstacleDistance, 1);
assertClose(result.Scores.LaneCenteringAccuracy, 1);
assertClose(result.TotalScore, 3.9);
end

function testUnknownOfficialGeometryMetrics()
telemetry = makeTelemetry(0:0.1:1);
telemetry.task_complete(end) = 1;
events = makeLaneChangeEvent(true);

result = scoreNavigationRun(telemetry, events, TMin_s=0, TMax_s=2);
unknown = sort(result.UnknownMetrics);

assert(isequal(unknown, sort([
    "LaneMarkingCompliance"
    "MinimumObstacleDistance"
    ])));
assertClose(result.KnownMaxPoints, 55);
assert(isnan(result.Scores.RedSignalCompliance));
end

function testSignalStatesIgnored()
telemetry = makeTelemetry(0:0.1:1);
telemetry.signal_detected = [];
events = struct( ...
    "timestamp_s", 0.5, ...
    "event_type", "intersection_entry", ...
    "ego_x", 0.0, ...
    "ego_y", 0.0, ...
    "details", struct("intersection_id", "I1"));

result = scoreNavigationRun(telemetry, events, ...
    TMin_s=0, ...
    TMax_s=2, ...
    YellowLineViolationCount=0, ...
    MinimumObstacleDistance_m=1.0);

assert(result.Validation.TelemetrySchemaValid);
assert(result.Validation.EventsSchemaValid);
assert(isequal(result.IgnoredMetrics, "RedSignalCompliance"));
end

function testTelemetrySchemaValidation()
telemetry = makeTelemetry(0:0.1:1);
telemetry.ego_speed_kmh = [];

try
    scoreNavigationRun(telemetry, []);
    error("runScoringTests:ExpectedFailure", "Schema validation should have failed.");
catch ME
    assert(strcmp(ME.identifier, "scoreNavigationRun:TelemetrySchemaInvalid"));
end
end

function telemetry = makeTelemetry(time_s)
time_s = time_s(:);
n = numel(time_s);
telemetry = table( ...
    time_s, ...
    zeros(n, 1), ...
    zeros(n, 1), ...
    zeros(n, 1), ...
    zeros(n, 1), ...
    zeros(n, 1), ...
    repmat("none", n, 1), ...
    repmat(2.1, n, 1), ...
    repmat(1.4, n, 1), ...
    repmat("rear_axle_center", n, 1), ...
    zeros(n, 1), ...
    repmat("none", n, 1), ...
    zeros(n, 1), ...
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
        "signal_detected", ...
        "task_complete"]);
end

function events = makeLaneChangeEvent(indicatorActive)
events = struct( ...
    "timestamp_s", 1.0, ...
    "event_type", "lane_change_start", ...
    "ego_x", 0.0, ...
    "ego_y", 0.0, ...
    "details", struct( ...
        "from_lane", "R1_L1", ...
        "to_lane", "R1_L2", ...
        "indicator_active", indicatorActive));
end

function assertClose(actual, expected)
tol = 1e-9;
assert(abs(actual - expected) <= tol, ...
    "Expected %.12g, got %.12g.", expected, actual);
end

function writeText(filePath, text)
fid = fopen(filePath, "w");
assert(fid ~= -1, "Could not open temporary file for writing.");
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", text);
end

function deleteIfExists(paths)
for i = 1:numel(paths)
    if isfile(paths{i})
        delete(paths{i});
    end
end
end
