# MathWorks Advanced Simulation Framework

This repository contains the Simulink model used for the aBAJA 2026 advanced simulation workflow.

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

## How to Run

1. Open MATLAB.
2. Open the project:

   ```matlab
   openProject("MathWorksAdvancedSimulation_Framework.prj")
   ```

3. Wait for project setup to finish. It skips the download if `localFolder` already contains the required files.
4. Open the model:

   ```matlab
   open_system("ControlVehicle.slx")
   ```

5. Run the model from Simulink.

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
