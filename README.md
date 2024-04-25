# EMBER, FRC 4451's Crescendo Robot

## Prerequisites

- Latest version of WPILib
- VSCode or other editor with WPILib installed
- AdvantageScope, if not installed with WPILib

## Getting Started

1. Clone this repository
2. Open the codebase using the WPILib version of VS Code from the WPILib installer
3. Run `WPILib: Build Robot Code` to generate your build constants for AdvantageKit

## Simulating robot code

Our robot code was built entirely with simulation-first development. We utilized AdvantageKit's structure to build out the subsystems, then IO layer, then Sim layer, then motor controller(s) for each subsystem. Some components, such as the `pivot` and `elevator` subsystems also leverage the `Mechanism2d` construct to help us visualize angle/height setpoints to ensure robot parts will not collide.

Assuming you have built robot code, you can run `WPILib: Simulate Robot Code` to start the WPILib simulator and observe our logged properties in AdvantageScope.

- You can read more on simulation controls from [WPILib](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)

When running the robot in `SIM` mode, `PhotonSim` is enabled and updated for each April Tag and Object Detection camera established in the codebase.

- You can read more on PhotonSim from [PhotonVision](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html#visualizing-results)

From our robot code, ports 1181\~1188 are used by the four April Tag cameras. Ports 1189\~1190 are used for Object Detection. If you add/remove cameras, expect the port designations to change.
