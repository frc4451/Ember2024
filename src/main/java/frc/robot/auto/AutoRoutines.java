package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.pivot.PivotLocation;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.RollerMode;
import frc.robot.subsystems.RollerSubsystem;

public class AutoRoutines {
    private static Command runCommandForSeconds(Command cmd, double seconds) {
        return new WaitCommand(seconds).deadlineWith(cmd.repeatedly());
    }

    private static Command runCommandAfterSeconds(Command cmd, double seconds) {
        return new WaitCommand(seconds).andThen(cmd);
    }

    public static Command getTestRoutine(DriveSubsystem drive, List<PathPlannerTrajectory> paths) {
        return drive.followTrajectoryCommand(paths.get(0), true, true);
    }

    public static Command getTriPoop(
            List<PathPlannerTrajectory> paths,
            DriveSubsystem drive,
            RollerSubsystem rollers,
            PivotSubsystem pivot) {
        final PathPlannerTrajectory goGrabSecond = paths.get(0);
        final PathPlannerTrajectory goPoopSecond = paths.get(1);
        final PathPlannerTrajectory goGrabThird = paths.get(2);
        final PathPlannerTrajectory goPoopThird = paths.get(3);

        return new ParallelCommandGroup(
                pivot.pivotCommand().repeatedly(),
                new SequentialCommandGroup(
                        // poop first game piece
                        runCommandForSeconds(rollers.runRollersCommand(RollerMode.SHOOT_LOW), 0.5),
                        // go to second piece and grab it
                        new ParallelDeadlineGroup(
                                drive.followTrajectoryCommand(goGrabSecond, true, true),
                                runCommandAfterSeconds(
                                        pivot.setSetpointCommand(PivotLocation.k0.angle),
                                        1.0),
                                rollers.runRollersCommand(RollerMode.SUCK).repeatedly()),
                        // go back and poop second game piece
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        drive.followTrajectoryCommand(goPoopSecond, false, true),
                                        runCommandForSeconds(rollers.runRollersCommand(RollerMode.SHOOT_LOW), 0.25)),
                                pivot.setSetpointCommand(PivotLocation.k167.angle)),
                        // actually grab third piece
                        new ParallelDeadlineGroup(
                                drive.followTrajectoryCommand(goGrabThird, true, true),
                                runCommandAfterSeconds(
                                        pivot.setSetpointCommand(PivotLocation.k0.angle),
                                        1.0),
                                rollers.runRollersCommand(RollerMode.SUCK).repeatedly()),
                        // poop third game piece
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        drive.followTrajectoryCommand(goPoopThird, false, true),
                                        runCommandForSeconds(rollers.runRollersCommand(RollerMode.SHOOT_LOW), 0.25),
                                        rollers.runRollersCommand(RollerMode.STOP)),
                                pivot.setSetpointCommand(PivotLocation.k167.angle))));
    }

    public static Command getBiPoop(
            List<PathPlannerTrajectory> paths,
            DriveSubsystem drive,
            RollerSubsystem rollers,
            PivotSubsystem pivot) {
        final PathPlannerTrajectory goGrabSecond = paths.get(0);
        final PathPlannerTrajectory goPoopSecond = paths.get(1);
        final PathPlannerTrajectory goGrabThird = paths.get(2);

        return new ParallelCommandGroup(
                pivot.pivotCommand().repeatedly(),
                new SequentialCommandGroup(
                        // poop first game piece
                        runCommandForSeconds(rollers.runRollersCommand(RollerMode.SHOOT_LOW), 0.5),
                        // go to second piece and grab it
                        new ParallelDeadlineGroup(
                                drive.followTrajectoryCommand(goGrabSecond, true, true),
                                runCommandAfterSeconds(
                                        pivot.setSetpointCommand(PivotLocation.k0.angle),
                                        1.0),
                                rollers.runRollersCommand(RollerMode.SUCK).repeatedly()),
                        // go back and poop second game piece
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        drive.followTrajectoryCommand(goPoopSecond, false, true),
                                        runCommandForSeconds(rollers.runRollersCommand(RollerMode.SHOOT_LOW), 0.25)),
                                pivot.setSetpointCommand(PivotLocation.k167.angle)),
                        // actually grab third piece
                        new ParallelDeadlineGroup(
                                drive.followTrajectoryCommand(goGrabThird, true, true),
                                runCommandAfterSeconds(
                                        pivot.setSetpointCommand(PivotLocation.k0.angle),
                                        1.0),
                                rollers.runRollersCommand(RollerMode.SUCK).repeatedly())));
    }

    public static Command getCenterPoopOver(
            List<PathPlannerTrajectory> paths,
            DriveSubsystem drive,
            RollerSubsystem rollers) {
        return new SequentialCommandGroup(
                runCommandForSeconds(rollers.runRollersCommand(RollerMode.SHOOT_LOW), 0.25),
                rollers.runRollersCommand(RollerMode.STOP),
                drive.followTrajectoryCommand(
                        paths.get(0),
                        true,
                        true),
                new RunCommand(drive::setCross, drive).repeatedly());
    }

}
