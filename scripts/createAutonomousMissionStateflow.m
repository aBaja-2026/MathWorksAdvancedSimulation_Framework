function createAutonomousMissionStateflow
% createAutonomousMissionStateflow Create the guideline-level mission chart.
%
% The chart is passive documentation. It has no Simulink ports and no control
% outputs. Guard variables are local chart data that default to zero, so the
% chart does not affect the current vehicle-control algorithm.

modelName = "ControlVehicle";
chartName = "Autonomous Mission Stateflow";
chartBlock = modelName + "/" + chartName;

load_system(modelName);
deleteExistingMissionCharts(modelName, chartName);

add_block("sflib/Chart", chartBlock, ...
    "Position", [650 520 1260 920]);

rt = sfroot;
chartId = sfprivate("block2chart", get_param(chartBlock, "Handle"));
chart = rt.idToHandle(chartId);
chart.ActionLanguage = "MATLAB";
clearChartContents(chart);
addGuardData(chart);

% Startup and readiness states.
loadSetup = makeState(chart, "Load_Setup_Simulation", [40 70 170 55]);
validateSensors = makeState(chart, "Validate_Sensor_Config", [250 70 170 55]);
localizeStart = makeState(chart, "Localize_At_Point_A", [460 70 170 55]);
plannerReady = makeState(chart, "Planner_Ready", [670 70 150 55]);

% Normal mission states.
parked = makeState(chart, "Parked_At_Point_A", [40 190 170 55]);
merge = makeState(chart, "Merge_From_Park", [250 190 170 55]);
laneFollowing = makeState(chart, "Lane_Following", [460 190 170 55]);
speedGuard = makeState(chart, "Speed_Limit_Guard", [460 300 170 55]);

% Traffic-signal and intersection states.
signalHandling = makeState(chart, "Traffic_Signal_Handling", [670 190 185 55]);
stopAtRed = makeState(chart, "Stop_At_Red", [895 190 150 55]);
intersection = makeState(chart, "Intersection_Traversal", [1085 190 185 55]);

% Obstacle and lane-change states.
obstacleAssessment = makeState(chart, "Obstacle_Assessment", [670 330 185 55]);
obstacleBypass = makeState(chart, "Obstacle_Bypass", [895 330 150 55]);
laneChange = makeState(chart, "Lane_Change_With_Indicator", [1085 330 185 55]);
returnLane = makeState(chart, "Return_To_Lane", [1085 440 185 55]);

% Destination and completion states.
destinationApproach = makeState(chart, "Destination_Approach", [670 500 185 55]);
destinationStop = makeState(chart, "Destination_Stop", [895 500 150 55]);
taskComplete = makeState(chart, "Task_Complete", [1085 500 150 55]);

% Fail-safe states.
emergencyBrake = makeState(chart, "Emergency_Brake", [250 650 170 55]);
collisionHold = makeState(chart, "Collision_Hold", [460 650 170 55]);
lostLocalization = makeState(chart, "Lost_Localization", [670 650 170 55]);
ruleViolationHold = makeState(chart, "Rule_Violation_Hold", [880 650 170 55]);

% Main flow.
makeDefaultTransition(chart, loadSetup, [20 98]);
makeTransition(chart, loadSetup, validateSensors, "[setupLoaded]");
makeTransition(chart, validateSensors, localizeStart, "[sensorsValid]");
makeTransition(chart, localizeStart, plannerReady, "[localizationReady]");
makeTransition(chart, plannerReady, parked, "[plannerReadyFlag]");
makeTransition(chart, parked, merge, "[missionStart]");
makeTransition(chart, merge, laneFollowing, "[laneAcquired]");

% Continuous driving rule handling.
makeTransition(chart, laneFollowing, speedGuard, "[speedKmh > 30]");
makeTransition(chart, speedGuard, laneFollowing, "[speedKmh <= 30]");
makeTransition(chart, laneFollowing, signalHandling, "[signalDetected]");
makeTransition(chart, signalHandling, stopAtRed, "[redSignalDetected]");
makeTransition(chart, signalHandling, intersection, "[greenSignalDetected]");
makeTransition(chart, stopAtRed, intersection, "[greenSignalDetected]");
makeTransition(chart, intersection, laneFollowing, "[intersectionClear]");

% Obstacle and lane-change handling.
makeTransition(chart, laneFollowing, obstacleAssessment, "[obstacleDetected]");
makeTransition(chart, obstacleAssessment, obstacleBypass, "[bypassPathReady]");
makeTransition(chart, obstacleBypass, laneChange, "[laneChangeRequired]");
makeTransition(chart, laneChange, returnLane, "[laneChangeComplete]");
makeTransition(chart, returnLane, laneFollowing, "[laneCentered]");

% Destination handling.
makeTransition(chart, laneFollowing, destinationApproach, "[nearDestination]");
makeTransition(chart, destinationApproach, destinationStop, "[insidePointBZone]");
makeTransition(chart, destinationStop, taskComplete, "[stoppedInTargetZone]");

% Fail-safe transitions from active vehicle motion states.
faultGuard = "[collisionDetected || localizationLost || ruleViolation]";
makeTransition(chart, merge, emergencyBrake, faultGuard);
makeTransition(chart, laneFollowing, emergencyBrake, faultGuard);
makeTransition(chart, speedGuard, emergencyBrake, faultGuard);
makeTransition(chart, signalHandling, emergencyBrake, faultGuard);
makeTransition(chart, stopAtRed, emergencyBrake, faultGuard);
makeTransition(chart, intersection, emergencyBrake, faultGuard);
makeTransition(chart, obstacleAssessment, emergencyBrake, faultGuard);
makeTransition(chart, obstacleBypass, emergencyBrake, faultGuard);
makeTransition(chart, laneChange, emergencyBrake, faultGuard);
makeTransition(chart, returnLane, emergencyBrake, faultGuard);
makeTransition(chart, destinationApproach, emergencyBrake, faultGuard);
makeTransition(chart, destinationStop, emergencyBrake, faultGuard);
makeTransition(chart, emergencyBrake, collisionHold, "[collisionDetected]");
makeTransition(chart, emergencyBrake, lostLocalization, "[localizationLost]");
makeTransition(chart, emergencyBrake, ruleViolationHold, "[ruleViolation]");

addGuidelineAnnotation(modelName);
save_system(modelName);

fprintf("Created %s and saved %s.slx.\n", chartBlock, modelName);
end

function deleteExistingMissionCharts(modelName, chartName)
cleanupNames = [chartName, "Autonomous_Mission_Stateflow"];
for i = 1:numel(cleanupNames)
    existingBlocks = find_system(modelName, ...
        "SearchDepth", 1, ...
        "Name", cleanupNames(i));
    for j = 1:numel(existingBlocks)
        delete_block(existingBlocks{j});
    end
end
end

function clearChartContents(chart)
objects = chart.find("-depth", 1);
for i = 1:numel(objects)
    if objects(i) ~= chart
        try
            delete(objects(i));
        catch
        end
    end
end
end

function addGuardData(chart)
guardNames = [
    "setupLoaded"
    "sensorsValid"
    "localizationReady"
    "plannerReadyFlag"
    "missionStart"
    "laneAcquired"
    "signalDetected"
    "redSignalDetected"
    "greenSignalDetected"
    "intersectionClear"
    "obstacleDetected"
    "bypassPathReady"
    "laneChangeRequired"
    "laneChangeComplete"
    "laneCentered"
    "nearDestination"
    "insidePointBZone"
    "stoppedInTargetZone"
    "collisionDetected"
    "localizationLost"
    "ruleViolation"
    "speedKmh"
    ];

for i = 1:numel(guardNames)
    data = Stateflow.Data(chart);
    data.Name = guardNames(i);
    data.Scope = "Local";
    data.Props.Array.Size = "1";
    if guardNames(i) == "speedKmh"
        data.DataType = "double";
        data.Props.InitialValue = "0";
    else
        data.DataType = "boolean";
        data.Props.InitialValue = "false";
    end
end
end

function state = makeState(chart, name, position)
state = Stateflow.State(chart);
state.Name = name;
state.LabelString = name;
state.Position = position;
end

function transition = makeTransition(chart, source, destination, label)
transition = Stateflow.Transition(chart);
transition.Source = source;
transition.Destination = destination;
transition.LabelString = label;
end

function transition = makeDefaultTransition(chart, destination, sourceEndpoint)
transition = Stateflow.Transition(chart);
transition.Destination = destination;
transition.SourceEndpoint = sourceEndpoint;
transition.DestinationEndpoint = [
    destination.Position(1), ...
    destination.Position(2) + destination.Position(4) / 2];
end

function addGuidelineAnnotation(modelName)
annotationText = "Autonomous Mission Stateflow" + newline + ...
    "Guideline coverage: Point A startup, merge, lane follow, speed guard, signal stop, obstacle bypass, indicator lane change, destination stop, fail-safe." + newline + ...
    "Traffic constraints: <=30 km/h, stop at red, no solid-yellow crossing, no collisions, stop at Point B.";

try
    annotations = find_system(modelName, "FindAll", "on", "Type", "annotation");
    for i = 1:numel(annotations)
        plainText = string(get_param(annotations(i), "PlainText"));
        if startsWith(plainText, "Autonomous Mission Stateflow")
            delete(annotations(i));
        end
    end

    annotation = Simulink.Annotation(modelName, annotationText);
    annotation.Position = [650 455 1260 505];
catch
end
end
