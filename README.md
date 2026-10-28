# 9584 WCP Code 2026

This repository contains the code used for the 9584 version of the WestCoast Products 2026 [Competitive Concept](https://wcproducts.com/pages/wcp-competitive-concepts).

The project is based on one of CTRE's [Phoenix 6 example projects](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithChoreo). It uses WPILib [command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) to manage robot subsystems and actions, a [Limelight](https://limelightvision.io/) for vision, and [Pathplanner](https://pathplanner.dev/home.html) for autonomous path following.

**Simulation**
We use [MapleSim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) with [AdvantageScope](https://docs.advantagescope.org/) for simulation.
Using xbox controller to control in the simulation


To start up simulation:
1. Install MapleSim and AdvantageScope from their respective websites.
2. Run `./gradlew simulateJava` in the terminal or WPILib: Simulate Robot Code to start the simulation.
3. Open AdvantageScope and connect to the simulator. 
    * On Windows, either do File > Connect To Simulator > Default: NetworkTables 4 or press Ctrl+Shift+K to connect.
    * On Mac, press Ctrl+Shift+K to connect.
4. Once connected, in the sidebar expand `Pose` and drag `robotPose` into the Poses box at the bottom so the robot appears on the field.
5. In the AdvantageScope tree, expand `AdvantageKit > RealOutputs > Fuel Simulation` and drag `Fuels` into a graph or table to monitor simulated fuel levels alongside the pose.
6. Open `App > AdvantageScope` (left top corner) and set `Use Custom Assets Folder` to `~/9584-wcp-code-2026/ascope_assets` so the custom 9584 robot model loads in the viewer.
