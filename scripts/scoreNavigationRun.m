function result = scoreNavigationRun(telemetryInput, eventsInput, options)
% scoreNavigationRun Score an aBAJA Advanced Simulation navigation run.
%
% The function validates the required telemetry/events log schema and computes
% all guideline metrics that can be evaluated from logs alone. Signal-state
% scoring is ignored by default for the current project phase. Metrics that
% require the official evaluation geometry can be supplied as precomputed
% options: YellowLineViolationCount and MinimumObstacleDistance_m. If
% IgnoreSignalStates is false, RedSignalViolationCount is also scored.

arguments
    telemetryInput
    eventsInput = []
    options.TMin_s (1, 1) double = NaN
    options.TMax_s (1, 1) double = NaN
    options.RedSignalViolationCount (1, 1) double = NaN
    options.YellowLineViolationCount (1, 1) double = NaN
    options.MinimumObstacleDistance_m (1, 1) double = NaN
    options.StrictSchema (1, 1) logical = true
    options.IgnoreSignalStates (1, 1) logical = true
end

telemetry = readTelemetry(telemetryInput);
events = readEvents(eventsInput);

[schemaValid, schemaIssues] = validateTelemetrySchema(telemetry, options.StrictSchema, options.IgnoreSignalStates);
[eventsValid, eventIssues] = validateEventsSchema(events, options.IgnoreSignalStates);

time_s = telemetry.timestamp_s;
speed_kmh = telemetry.ego_speed_kmh;
lateralDeviation_m = telemetry.lateral_deviation_m;
collisionFlag = telemetry.collision_flag ~= 0;
taskCompleteFlag = telemetry.task_complete ~= 0;

taskComplete = any(taskCompleteFlag);
if taskComplete
    completionIndex = find(taskCompleteFlag, 1, "first");
    completionTime_s = time_s(completionIndex);
else
    completionIndex = NaN;
    completionTime_s = NaN;
end

metrics = struct;
metrics.TaskComplete = taskComplete;
metrics.CompletionIndex = completionIndex;
metrics.CompletionTime_s = completionTime_s;
metrics.OverspeedDuration_s = integrateRowDuration(time_s, speed_kmh > 30);
metrics.CollisionEventCount = countFlagEvents(collisionFlag);
metrics.IndicatorViolationCount = countIndicatorViolations(events);
metrics.LateralDeviationRMSE_m = sqrt(mean(lateralDeviation_m.^2, "omitnan"));
metrics.RedSignalViolationCount = options.RedSignalViolationCount;
metrics.YellowLineViolationCount = options.YellowLineViolationCount;
metrics.MinimumObstacleDistance_m = options.MinimumObstacleDistance_m;

scores = struct;
scores.TaskCompletion = 20 * double(taskComplete);
scores.NavigationTime = scoreNavigationTime(taskComplete, completionTime_s, options.TMin_s, options.TMax_s);
scores.CollisionPenalty = -5 * metrics.CollisionEventCount;
scores.SpeedLimitCompliance = max(0, 10 - metrics.OverspeedDuration_s);
if options.IgnoreSignalStates
    scores.RedSignalCompliance = NaN;
else
    scores.RedSignalCompliance = scoreCountPenalty(metrics.RedSignalViolationCount, 10, 5);
end
scores.LaneMarkingCompliance = scoreCountPenalty(metrics.YellowLineViolationCount, 10, 3);
scores.IndicatorCompliance = max(0, 5 - 2 * metrics.IndicatorViolationCount);
scores.MinimumObstacleDistance = scoreObstacleDistance(metrics.MinimumObstacleDistance_m);
scores.LaneCenteringAccuracy = scoreLaneCentering(metrics.LateralDeviationRMSE_m);

positiveNames = [
    "TaskCompletion"
    "NavigationTime"
    "SpeedLimitCompliance"
    "LaneMarkingCompliance"
    "IndicatorCompliance"
    "MinimumObstacleDistance"
    "LaneCenteringAccuracy"
    ];
ignoredMetrics = strings(0, 1);
if options.IgnoreSignalStates
    ignoredMetrics(end + 1, 1) = "RedSignalCompliance";
else
    positiveNames = [
        positiveNames(1:3)
        "RedSignalCompliance"
        positiveNames(4:end)
        ];
end
maxPoints = struct( ...
    "TaskCompletion", 20, ...
    "NavigationTime", 15, ...
    "SpeedLimitCompliance", 10, ...
    "RedSignalCompliance", 10, ...
    "LaneMarkingCompliance", 10, ...
    "IndicatorCompliance", 5, ...
    "MinimumObstacleDistance", 5, ...
    "LaneCenteringAccuracy", 5);

[knownPositiveScore, knownMaxPoints, unknownMetrics] = sumKnownScores(scores, positiveNames, maxPoints);
totalBeforeCollisionPenalty = knownPositiveScore;
totalScore = max(0, knownPositiveScore + scores.CollisionPenalty);

sampleDt = diff(time_s);
validation = struct;
validation.TelemetrySchemaValid = schemaValid;
validation.EventsSchemaValid = eventsValid;
validation.TimeMonotonic = all(sampleDt > 0);
validation.Minimum10Hz = isempty(sampleDt) || max(sampleDt) <= 0.1000001;
validation.Issues = [schemaIssues; eventIssues; timingIssues(validation)];

result = struct;
result.Metrics = metrics;
result.Scores = scores;
result.TotalKnownPositiveScore = knownPositiveScore;
result.TotalBeforeCollisionPenalty = totalBeforeCollisionPenalty;
result.TotalScore = totalScore;
result.KnownMaxPoints = knownMaxPoints;
result.UnknownMetrics = unknownMetrics;
result.IgnoredMetrics = ignoredMetrics;
result.Validation = validation;
result.MaxPoints = maxPoints;
end

function telemetry = readTelemetry(input)
if istable(input)
    telemetry = input;
    return
end

if isstring(input) || ischar(input)
    input = char(input);
    if ~isfile(input)
        error("scoreNavigationRun:TelemetryFileNotFound", ...
            "Telemetry log not found: %s", input);
    end
    telemetry = readtable(input, "TextType", "string");
    return
end

error("scoreNavigationRun:InvalidTelemetryInput", ...
    "telemetryInput must be a table or a telemetry_log.csv path.");
end

function events = readEvents(input)
emptyEvents = struct("timestamp_s", {}, "event_type", {}, "ego_x", {}, "ego_y", {}, "details", {});
if isempty(input)
    events = emptyEvents;
    return
end

if isstring(input) || ischar(input)
    input = char(input);
    if ~isfile(input)
        error("scoreNavigationRun:EventsFileNotFound", ...
            "Events log not found: %s", input);
    end
    text = strtrim(fileread(input));
    if strlength(string(text)) == 0
        events = emptyEvents;
    else
        events = jsondecode(text);
    end
elseif isstruct(input)
    events = input;
else
    error("scoreNavigationRun:InvalidEventsInput", ...
        "eventsInput must be a struct array or an events_log.json path.");
end

if isempty(events)
    events = emptyEvents;
end
end

function [valid, issues] = validateTelemetrySchema(telemetry, strictSchema, ignoreSignalStates)
requiredColumns = [
    "timestamp_s"
    "ego_x"
    "ego_y"
    "ego_speed_kmh"
    "ego_heading_deg"
    "lateral_deviation_m"
    "indicator_state"
    "vehicle_length_m"
    "vehicle_width_m"
    "ego_origin_ref"
    "collision_flag"
    "task_complete"
    ];
if ~ignoreSignalStates
    requiredColumns = [
        requiredColumns(1:11)
        "signal_detected"
        requiredColumns(12)
        ];
end

issues = strings(0, 1);
names = string(telemetry.Properties.VariableNames(:));
missing = setdiff(requiredColumns, names, "stable");
if ~isempty(missing)
    issues(end + 1, 1) = "Missing telemetry columns: " + strjoin(missing, ", ");
end

comparableNames = names;
if ignoreSignalStates
    comparableNames(comparableNames == "signal_detected") = [];
end
if strictSchema && numel(comparableNames) >= numel(requiredColumns)
    firstColumns = comparableNames(1:numel(requiredColumns));
    if ~isequal(firstColumns, requiredColumns)
        issues(end + 1, 1) = "Telemetry columns are not in the required order.";
    end
end

numericColumns = setdiff(requiredColumns, ["indicator_state", "ego_origin_ref", "signal_detected"], "stable");
for i = 1:numel(numericColumns)
    name = numericColumns(i);
    if any(names == name) && ~isnumeric(telemetry.(name))
        issues(end + 1, 1) = "Telemetry column must be numeric: " + name;
    end
end

valid = isempty(issues);
if ~valid
    error("scoreNavigationRun:TelemetrySchemaInvalid", "%s", strjoin(issues, newline));
end
end

function [valid, issues] = validateEventsSchema(events, ignoreSignalStates)
issues = strings(0, 1);
if isempty(events)
    valid = true;
    return
end

requiredFields = ["timestamp_s", "event_type", "ego_x", "ego_y", "details"];
for i = 1:numel(requiredFields)
    if ~isfield(events, requiredFields(i))
        issues(end + 1, 1) = "Missing event field: " + requiredFields(i);
    end
end

if ~isempty(issues)
    valid = false;
    error("scoreNavigationRun:EventsSchemaInvalid", "%s", strjoin(issues, newline));
end

allowedTypes = [
    "lane_change_start"
    "lane_change_end"
    "lane_following"
    "right_turn"
    "left_turn"
    "intersection_entry"
    "intersection_exit"
    "collision_start"
    "collision_end"
    ];

for i = 1:numel(events)
    eventType = string(events(i).event_type);
    if ~any(eventType == allowedTypes)
        issues(end + 1, 1) = "Unknown event_type: " + eventType;
        continue
    end
    detailIssues = validateEventDetails(eventType, events(i).details, ignoreSignalStates);
    issues = [issues; detailIssues]; %#ok<AGROW>
end

valid = isempty(issues);
if ~valid
    error("scoreNavigationRun:EventsSchemaInvalid", "%s", strjoin(issues, newline));
end
end

function issues = validateEventDetails(eventType, details, ignoreSignalStates)
issues = strings(0, 1);
if ~isstruct(details)
    issues(end + 1, 1) = "details must be an object for event_type: " + eventType;
    return
end

switch eventType
    case "lane_change_start"
        required = ["from_lane", "to_lane", "indicator_active"];
    case "lane_change_end"
        required = ["from_lane", "to_lane", "success"];
    case "lane_following"
        required = "ego_speed_kmh";
    case {"right_turn", "left_turn"}
        required = ["intersection_id", "ego_speed_kmh"];
    case "intersection_entry"
        if ignoreSignalStates
            required = "intersection_id";
        else
            required = ["intersection_id", "signal_state_detected"];
        end
    case "intersection_exit"
        required = "intersection_id";
    case "collision_start"
        required = ["obstacle_id", "ego_speed_kmh"];
    case "collision_end"
        required = ["obstacle_id", "duration_s"];
    otherwise
        required = strings(0, 1);
end

for i = 1:numel(required)
    if ~isfield(details, required(i))
        issues(end + 1, 1) = "Missing details." + required(i) + " for event_type: " + eventType;
    end
end
end

function duration_s = integrateRowDuration(time_s, mask)
if numel(time_s) < 2
    duration_s = 0;
    return
end

dt = diff(time_s(:));
mask = mask(:);
duration_s = sum(dt(mask(1:end - 1)), "omitnan");
end

function count = countFlagEvents(flag)
flag = flag(:);
if isempty(flag)
    count = 0;
    return
end

starts = flag & [true; ~flag(1:end - 1)];
count = nnz(starts);
end

function count = countIndicatorViolations(events)
count = 0;
for i = 1:numel(events)
    if string(events(i).event_type) == "lane_change_start"
        details = events(i).details;
        if isfield(details, "indicator_active") && ~logical(details.indicator_active)
            count = count + 1;
        end
    end
end
end

function score = scoreNavigationTime(taskComplete, completionTime_s, tMin_s, tMax_s)
if ~taskComplete
    score = 0;
elseif ~isfinite(tMin_s) || ~isfinite(tMax_s) || tMax_s <= tMin_s
    score = NaN;
else
    score = 15 * (1 - (completionTime_s - tMin_s) / (tMax_s - tMin_s));
    score = min(15, max(0, score));
end
end

function score = scoreCountPenalty(count, maxScore, penaltyPerEvent)
if ~isfinite(count)
    score = NaN;
else
    score = max(0, maxScore - penaltyPerEvent * count);
end
end

function score = scoreObstacleDistance(distance_m)
if ~isfinite(distance_m)
    score = NaN;
elseif distance_m >= 1.0
    score = 5;
elseif distance_m >= 0.5
    score = 3;
elseif distance_m >= 0.2
    score = 1;
else
    score = 0;
end
end

function score = scoreLaneCentering(rmse_m)
if ~isfinite(rmse_m)
    score = NaN;
elseif rmse_m <= 0.10
    score = 5;
elseif rmse_m <= 0.20
    score = 3;
elseif rmse_m <= 0.35
    score = 1;
else
    score = 0;
end
end

function [knownScore, knownMaxPoints, unknownMetrics] = sumKnownScores(scores, positiveNames, maxPoints)
knownScore = 0;
knownMaxPoints = 0;
unknownMetrics = strings(0, 1);

for i = 1:numel(positiveNames)
    name = positiveNames(i);
    value = scores.(name);
    if isfinite(value)
        knownScore = knownScore + value;
        knownMaxPoints = knownMaxPoints + maxPoints.(name);
    else
        unknownMetrics(end + 1, 1) = name; %#ok<AGROW>
    end
end
end

function issues = timingIssues(validation)
issues = strings(0, 1);
if ~validation.TimeMonotonic
    issues(end + 1, 1) = "Telemetry timestamp_s must be strictly increasing.";
end
if ~validation.Minimum10Hz
    issues(end + 1, 1) = "Telemetry sample spacing exceeds 0.1 s, below the required 10 Hz minimum.";
end
end
