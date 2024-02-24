// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AdvantageKitConstants;
import frc.utils.VirtualSubsystem;

/**
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autoCommand;
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick (default is "/U/logs on RIO")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // Logger.addDataReceiver(
                // new WPILOGWriter("/home/lewis/log_dir")); // Log to my pc for testing replay
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                String path = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(path));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
                setUseTiming(false); // Run as fast as possible since we're replaying a log
                break;
        }

        DriverStation.silenceJoystickConnectionWarning(true);

        // Start AdvantageKit Logger
        Logger.registerURCL(URCL.startExternal());
        Logger.start();

        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        VirtualSubsystem.runPeriodically();
        CommandScheduler.getInstance().run();
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        // m_robotContainer.field.setRobotPose(m_robotContainer.m_robotDrive.getPose());
        m_robotContainer.field.setRobotPose(m_robotContainer.m_robotDrive.getPose());

        SmartDashboard.putNumber("X", m_robotContainer.m_robotDrive.getPose().getX());
        SmartDashboard.putNumber("Y", m_robotContainer.m_robotDrive.getPose().getY());
        SmartDashboard.putNumber("Pose Rotation", m_robotContainer.m_robotDrive.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Heading", m_robotContainer.m_robotDrive.getHeading().getDegrees());
        // SmartDashboard.putBoolean("Roller Beambreak Activated",
        // m_robotContainer.m_rollers.isBeamBreakActivated());
        // SmartDashboard.putNumber("Arm Pivot Deg",
        // m_robotContainer.m_pivot.getAngle().getDegrees());
        // SmartDashboard.putNumber("Arm Pivot Rad",
        // m_robotContainer.m_pivot.getAngle().getRadians());
        // SmartDashboard.putNumber("Arm Pivot Setpoint Deg",
        // m_robotContainer.m_pivot.getSetpoint().getDegrees());
        SmartDashboard.putData("Field", m_robotContainer.field);

        // if (m_robotContainer.m_autoChooser.get() != new InstantCommand()) {
        // String autonomousName = m_robotContainer.m_autoChooser.get().getName();
        // Logger.recordOutput("Autonomous/AutoName", autonomousName);
        // Logger.recordOutput("Autonomous/StartingPose",
        // PathPlannerAuto.getStaringPoseFromAutoFile(autonomousName));
        // }
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (m_robotContainer.m_driverController.getHID().getBButtonPressed()) {
            m_robotContainer.m_robotDrive.zeroHeading();
            m_robotContainer.m_robotDrive.resetPose(new Pose2d());
            // m_robotContainer.m_pivot.setAngle(PivotLocation.INITIAL.angle);
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autoCommand = m_robotContainer.m_autoChooser.get();

        if (m_autoCommand != null) {
            m_autoCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        VisionConstants.VISION_SYSTEM_SIM
                .ifPresent(visionSystemSim -> visionSystemSim.addAprilTags(VisionConstants.FIELD_LAYOUT));

        m_robotContainer.m_vision.robotPoseSupplier = m_robotContainer.m_robotDrive::getPose;
    }

    @Override
    public void simulationPeriodic() {
        VirtualSubsystem.runSimulationPeriodically();
    }
}
