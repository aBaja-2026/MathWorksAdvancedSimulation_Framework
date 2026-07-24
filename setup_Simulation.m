% setup_Simulation
% Defines the start and end points used by ControlVehicle.slx.

modelName = "ControlVehicle";
vehicleBlockName = "Simulation 3D Physics Vehicle";
vehicleBlock = modelName + "/" + vehicleBlockName;

carInitialPosition = [-24.022962, -16.420, 0];
carInitialRotation = [0, 0, 3.147965];
carEndPoint = [102.791, -142.687, 0.169998];
recommendedEndPointDistance_m = 200;
carStartToEndDistance_m = norm(carEndPoint - carInitialPosition);
simulationStopTime_s = 60;

if carStartToEndDistance_m < recommendedEndPointDistance_m
    warning("setup_Simulation:EndPointBelowRecommendedDistance", ...
        "The configured end point is %.3f m from the start point. The guideline target is at least %.3f m.", ...
        carStartToEndDistance_m, recommendedEndPointDistance_m);
end

carInitialPose = struct( ...
    "Model", modelName, ...
    "VehicleBlock", vehicleBlock, ...
    "Position", carInitialPosition, ...
    "Rotation", carInitialRotation, ...
    "EndPoint", carEndPoint, ...
    "StartToEndDistance_m", carStartToEndDistance_m, ...
    "SimulationStopTime_s", simulationStopTime_s);

fprintf("Initial car position [x y z] = [%.6f %.6f %.6f] m\n", carInitialPosition);
fprintf("Initial car rotation [roll pitch yaw] = [%.6f %.6f %.6f] rad\n", carInitialRotation);
fprintf("End point [x y z] = [%.6f %.6f %.6f] m\n", carEndPoint);
fprintf("Start-to-end straight-line distance = %.3f m\n", carStartToEndDistance_m);
fprintf("Simulation stop time = %.3f s\n", simulationStopTime_s);
