# MathWorks Advanced Simulation Framework

This repository contains the Simulink model used for the aBAJA 2026 advanced simulation workflow.

## Expected Workflow

1. Open MATLAB.
2. Open the project:

   ```matlab
   openProject("MathWorksAdvancedSimulation_Framework.prj")
   ```

3. Wait for project startup to finish. The startup scripts download missing local assets and open `ControlVehicle.slx`.
4. The model loads the previously saved vehicle spawn by default.
5. To create a new spawn point, click the `Create new spawn point` annotation in `ControlVehicle.slx`.
6. A dialog shows `Starting creation of spawn point`.
7. The RRHD road-point selector runs quietly, updates the `Simulation 3D Physics Vehicle` block, and saves the model.
8. A dialog shows `Finished spawn point generation`.
9. Run the simulation from Simulink.

The spawn button writes these block parameters automatically:

- `InitialPos`
- `InitialRot`

`InitialRot` is written in radians, matching the `Simulation 3D Physics Vehicle` block.

## Model

- `ControlVehicle.slx`

The model runs a 3D vehicle simulation with:

- a physics-based vehicle
- camera sensor
- radar sensor
- GPS sensor
- IMU sensor
- an `Algorithm` subsystem for vehicle control commands

The current `Algorithm` subsystem is a starter placeholder. Teams should replace or update it with their own control logic.

## Required Software

Use MATLAB/Simulink R2026a or a compatible later release.

Required MathWorks products:

- MATLAB
- Simulink
- Vehicle Dynamics Blockset
- Automated Driving Toolbox
- Sensor Fusion and Tracking Toolbox
- Navigation Toolbox

## Required Local Assets

The model expects the 3D simulation assets to be available locally.

When you open the MATLAB project, setup runs automatically and downloads the assets only if they are missing.

Download them from MATLAB:

```matlab
downloadAssets
```

This creates the ignored `localFolder` directory in this repository.

Default paths used by the model after setup:

- Unreal executable: `localFolder\Windows\AutoVrtlEnv\Binaries\Win64\AutoVrtlEnv.exe`
- OpenDRIVE file: `localFolder\IndianCityBlock.xodr`

If these files are in a different location, update the `Simulation 3D Scene Configuration` block in `ControlVehicle.slx`.

## Spawn Point Generation

The spawn workflow uses `localFolder\IndianCityBlock.rrhd` and the scripts in `scripts`.

The one-click Simulink button calls:

```matlab
lastRRHDSpawnUpdate = updateControlVehicleSpawnFromRRHD(500, [], [], modelName, opts);
```

This selects a start point and a point 500 m away on the RRHD lane network, converts the RRHD/RoadRunner pose to the Simulation 3D coordinate system, and updates the vehicle block.

The coordinate conversion applied to the initial vehicle pose is:

- position: `[x, y, z] -> [x, -y, z]`
- yaw: `wrapTo2Pi(3*pi/2 - deg2rad(rrhdYawDeg))`

The latest spawn result is also assigned in the MATLAB base workspace as `lastRRHDSpawnUpdate`.

For deterministic testing from MATLAB:

```matlab
opts = struct( ...
    "Quiet", true, ...
    "ShowDialogs", false, ...
    "CreatePlot", false, ...
    "SaveResult", false, ...
    "Fast", true, ...
    "VerticalStartOnly", true, ...
    "MapEdgeMargin", 50, ...
    "StartStationRandomWindow", 60, ...
    "StartStationAttempts", 12);

lastRRHDSpawnUpdate = updateControlVehicleSpawnFromRRHD(500, [], [], "ControlVehicle", opts);
```

## Validation

Run the coordinate-conversion check with:

```matlab
testConvertRRHDPoseToSim3DInitialPose
```

## What Teams Need To Do

Update the `Algorithm` subsystem.

Input:

- `SensorBus`

Outputs:

- `Steering`
- `AccelCmd`
- `DecelCmd`

Before submitting changes, make sure:

- the model opens without errors
- the 3D scene starts correctly
- the vehicle appears in the scene
- the algorithm produces valid steering, acceleration, and braking commands
