function configureScoringSignalLogging(modelName, options)
% configureScoringSignalLogging Enable raw signal logging needed for scoring.

arguments
    modelName (1, 1) string = "ControlVehicle"
    options.SaveModel (1, 1) logical = false
end

load_system(modelName);

vehicleBlock = modelName + "/Simulation 3D Physics Vehicle";
lineHandles = get_param(vehicleBlock, "LineHandles");
vehicleInfoLine = lineHandles.Outport(1);
if vehicleInfoLine == -1
    error("configureScoringSignalLogging:MissingVehicleInfoLine", ...
        "The vehicle Vehicle_Info output is not connected.");
end

set_param(vehicleInfoLine, "Name", "Vehicle_Info_Log");
Simulink.sdi.markSignalForStreaming(vehicleInfoLine, "on");

set_param(modelName, ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on");

if options.SaveModel
    save_system(modelName);
end
end
