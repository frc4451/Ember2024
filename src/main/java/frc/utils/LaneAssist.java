package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotDriveMode;

public record LaneAssist(Command pathfindCommand, Command aimingCommand) {
    public Command getPathfindCommand() {
        return Commands.deferredProxy(this::pathfindCommand)
                .beforeStarting(BobotDriveMode.CRUISE_CONTROL.updateDriveModeCommand())
                // Releasing the trigger doesn't finish the sequence so don't use commands
                .finallyDo(BobotDriveMode.TELEOP::updateDriveMode);
    }

    public Command getAimingCommand() {
        return Commands.deferredProxy(this::aimingCommand)
                .beforeStarting(BobotDriveMode.LANE_ASSIST.updateDriveModeCommand())
                .finallyDo(BobotDriveMode.TELEOP::updateDriveMode);
    }
};
