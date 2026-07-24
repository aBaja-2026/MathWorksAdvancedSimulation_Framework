function [simOut, telemetry, events, score] = runScoringSimulation(options)
% runScoringSimulation Run ControlVehicle and generate scoring logs.
%
% Example:
%   [simOut, telemetry, events, score] = runScoringSimulation(StopTime="5");

arguments
    options.ModelName (1, 1) string = "ControlVehicle"
    options.StopTime (1, 1) string = "5"
    options.OutputFolder (1, 1) string = fullfile(pwd, "results", "logs")
    options.SampleTime_s (1, 1) double = 0.1
    options.PointBRadius_m (1, 1) double = 5
    options.VehicleLength_m (1, 1) double = 2.1
    options.VehicleWidth_m (1, 1) double = 1.4
    options.GenerateLaneChangeEvents (1, 1) logical = false
    options.TMin_s (1, 1) double = NaN
    options.TMax_s (1, 1) double = NaN
    options.YellowLineViolationCount (1, 1) double = NaN
    options.MinimumObstacleDistance_m (1, 1) double = NaN
end

repoRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repoRoot);
addpath(fullfile(repoRoot, "scripts"));
cd(repoRoot);

setup_Simulation;
configureScoringSignalLogging(options.ModelName);

simOut = sim(options.ModelName, "StopTime", options.StopTime);
[telemetry, events, score] = generateScoringLogsFromSimOut(simOut, ...
    OutputFolder=options.OutputFolder, ...
    SampleTime_s=options.SampleTime_s, ...
    PointB=carEndPoint, ...
    PointBRadius_m=options.PointBRadius_m, ...
    VehicleLength_m=options.VehicleLength_m, ...
    VehicleWidth_m=options.VehicleWidth_m, ...
    GenerateLaneChangeEvents=options.GenerateLaneChangeEvents, ...
    TMin_s=options.TMin_s, ...
    TMax_s=options.TMax_s, ...
    YellowLineViolationCount=options.YellowLineViolationCount, ...
    MinimumObstacleDistance_m=options.MinimumObstacleDistance_m);

fprintf("Telemetry log: %s\n", score.OutputFiles.Telemetry);
fprintf("Events log:    %s\n", score.OutputFiles.Events);
fprintf("Score summary: %s\n", score.OutputFiles.Score);
fprintf("Known score:   %.3f / %.3f\n", score.TotalScore, score.KnownMaxPoints);
end
