function outputs = updateControlVehicleSpawnFromRRHD(distanceMeters, randomSeed, rrhdFile, modelName, options)
% updateControlVehicleSpawnFromRRHD Select RRHD road points and update ControlVehicle spawn.
%
% This is intended to be called from the clickable annotation in
% ControlVehicle.slx. It selects Point A and Point B, uses the first route
% segment to compute the RRHD/RoadRunner heading at Point A, converts that
% pose to Simulation 3D initial position/rotation, and writes the values to
% the Simulation 3D Physics Vehicle block.

if nargin < 1 || isempty(distanceMeters)
    distanceMeters = 500;
end

if nargin < 2
    randomSeed = [];
end

if nargin < 3
    rrhdFile = [];
end

if nargin < 4 || strlength(string(modelName)) == 0
    modelName = "ControlVehicle";
else
    modelName = string(modelName);
end

if nargin < 5 || isempty(options)
    options = struct;
end
quiet = getOption(options, "Quiet", false);
showDialogs = getOption(options, "ShowDialogs", false);
createPlot = getOption(options, "CreatePlot", ~quiet);
saveResult = getOption(options, "SaveResult", ~quiet);
fast = getOption(options, "Fast", quiet);
verticalStartOnly = getOption(options, "VerticalStartOnly", quiet);
mapEdgeMargin = getOption(options, "MapEdgeMargin", 50);
startStationRandomWindow = getOption(options, "StartStationRandomWindow", 60);
startStationAttempts = getOption(options, "StartStationAttempts", 12);

startDialog = [];
if showDialogs
    startDialog = msgbox("Starting creation of spawn point", ...
        "Create new spawn point", "help", "non-modal");
    drawnow;
end

try
    selectionOptions = struct( ...
        "Verbose", ~quiet, ...
        "CreatePlot", createPlot, ...
        "SaveResult", saveResult, ...
        "Fast", fast, ...
        "VerticalStartOnly", verticalStartOnly, ...
        "MapEdgeMargin", mapEdgeMargin, ...
        "StartStationRandomWindow", startStationRandomWindow, ...
        "StartStationAttempts", startStationAttempts);
    [pointA, pointB, selection] = selectRRHDRoadPoints( ...
        rrhdFile, distanceMeters, randomSeed, selectionOptions);
    rrhdYawDeg = routeStartYaw(selection.RoutePolyline);
    [initialPos, initialRot, transformDetails] = convertRRHDPoseToSim3DInitialPose(pointA, rrhdYawDeg);

    load_system(modelName);
    vehicleBlock = modelName + "/Simulation 3D Physics Vehicle";
    set_param(vehicleBlock, "InitialPos", mat2str(initialPos, 8));
    set_param(vehicleBlock, "InitialRot", mat2str(initialRot, 8));
    save_system(modelName);

    outputs = struct;
    outputs.PointA = pointA;
    outputs.PointB = pointB;
    outputs.RRHDYawDeg = rrhdYawDeg;
    outputs.InitialPos = initialPos;
    outputs.InitialRot = initialRot;
    outputs.Selection = selection;
    outputs.TransformDetails = transformDetails;
    outputs.Model = modelName;
    outputs.VehicleBlock = vehicleBlock;

    assignin("base", "lastRRHDSpawnUpdate", outputs);

    if quiet
        if showDialogs
            closeDialogIfValid(startDialog);
            msgbox("Finished spawn point generation", ...
                "Create new spawn point", "help", "modal");
        end
    else
        fprintf("\nUpdated %s spawn from RRHD road point selection\n", modelName);
        fprintf("  PointA:     [%.6f, %.6f, %.6f]\n", pointA);
        fprintf("  PointB:     [%.6f, %.6f, %.6f]\n", pointB);
        fprintf("  RRHD yaw:   %.6f deg\n", rrhdYawDeg);
        fprintf("  InitialPos: [%.6f, %.6f, %.6f]\n", initialPos);
        fprintf("  InitialRot: [%.6f, %.6f, %.6f] rad\n", initialRot);
        if strlength(selection.PlotFile) > 0
            fprintf("  Plot:       %s\n", selection.PlotFile);
        end
        fprintf("  Outputs also assigned to base workspace variable lastRRHDSpawnUpdate.\n");
        if showDialogs
            closeDialogIfValid(startDialog);
            msgbox("Finished spawn point generation", ...
                "Create new spawn point", "help", "modal");
        end
    end
catch ME
    closeDialogIfValid(startDialog);
    if showDialogs
        errordlg(ME.message, "Create new spawn point failed", "modal");
    end
    rethrow(ME);
end
end

function value = getOption(options, name, defaultValue)
if isstruct(options) && isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function closeDialogIfValid(dialogHandle)
if ~isempty(dialogHandle) && isgraphics(dialogHandle)
    close(dialogHandle);
end
end

function yawDeg = routeStartYaw(routePolyline)
if size(routePolyline, 1) < 2
    error("updateControlVehicleSpawnFromRRHD:ShortRoute", ...
        "Route polyline must contain at least two points to compute heading.");
end

for i = 1:size(routePolyline, 1) - 1
    delta = routePolyline(i + 1, 1:2) - routePolyline(i, 1:2);
    if norm(delta) > 1e-9
        yawDeg = atan2d(delta(2), delta(1));
        return
    end
end

error("updateControlVehicleSpawnFromRRHD:ZeroLengthRoute", ...
    "Route polyline has no nonzero first segment to compute heading.");
end
