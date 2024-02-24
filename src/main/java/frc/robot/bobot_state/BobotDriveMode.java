package frc.robot.bobot_state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public enum BobotDriveMode {
    /** Drive Team Controlled (Default) */
    TELEOP,

    /** PathPlanner Trajectory (IE Auto) */
    TRAJECTORY,

    /** Following OTF path generated from PathPlanner */
    CRUISE_CONTROL,

    /** Using relative targeting from PhotonVision */
    LANE_ASSIST,
    ;

    public void updateDriveMode() {
        BobotState.updateDriveMode(this);
    }

    public Command updateDriveModeCommand() {
        return new InstantCommand(this::updateDriveMode);
    }
}
