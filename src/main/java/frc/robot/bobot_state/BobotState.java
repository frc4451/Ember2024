package frc.robot.bobot_state;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.apriltag.StageTags;
import frc.utils.VirtualSubsystem;

/**
 * Class full of static variables and methods that store robot state we'd need
 * across mulitple subsystems. It's called `BobotState` as to not conflict with
 * WPILib's {@link edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
    private static final String logRoot = "BobotState/";

    private static final ShootingInterpolator shootingInterpolator = new ShootingInterpolator();

    private static ShootingInterpolator.InterpolatedCalculation shootingCalculation;

    private static Pose2d robotPose = new Pose2d();

    static {
        shootingInterpolator.addEntries(
                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(10),
                        36.0,
                        65.0,
                        65.0),
                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(15),
                        31.0,
                        65.0,
                        65.0),
                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(21),
                        26.0,
                        85.0,
                        70.0));
    }

    public static void updateRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public static Pose2d getRobotPose() {
        return robotPose;
    }

    public static ShootingInterpolator.InterpolatedCalculation getShootingCalculation() {
        return shootingCalculation;
    }

    @Override
    public void periodic() {
        double distanceFromSpeaker = StageTags.SPEAKER_AIM.getDistanceFrom(robotPose);
        shootingCalculation = shootingInterpolator.calculateInterpolation(distanceFromSpeaker);

        {
            String calcLogRoot = logRoot + "ShootingCalculation/";
            Logger.recordOutput(calcLogRoot + "DistanceMeters", distanceFromSpeaker);
            Logger.recordOutput(calcLogRoot + "DistanceFeet", Units.metersToFeet(distanceFromSpeaker));
            Logger.recordOutput(calcLogRoot + "AngleDegrees", shootingCalculation.angleDegrees());
            Logger.recordOutput(calcLogRoot + "LeftSpeedRotPerSec", shootingCalculation.leftSpeedRotPerSec());
            Logger.recordOutput(calcLogRoot + "RightSpeedRotPerSec", shootingCalculation.rightSpeedRotPerSec());
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}
