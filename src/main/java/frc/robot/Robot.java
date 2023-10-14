// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RollerMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
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
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("X", m_robotContainer.m_robotDrive.getPose().getX());
        SmartDashboard.putNumber("Y", m_robotContainer.m_robotDrive.getPose().getY());
        SmartDashboard.putNumber("Pose Rotation",
                m_robotContainer.m_robotDrive.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Heading", m_robotContainer.m_robotDrive.getHeading());
        SmartDashboard.putNumber("Arm Pivot Deg", m_robotContainer.m_pivot.getAngle().getDegrees());
        SmartDashboard.putNumber("Arm Pivot Rad", m_robotContainer.m_pivot.getAngle().getRadians());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (m_robotContainer.m_driverController.b().getAsBoolean()) {
            m_robotContainer.m_robotDrive.zeroHeading();
            m_robotContainer.m_robotDrive.resetOdometry(new Pose2d());
        }
        // m_robotContainer.m_driverController.b().onTrue(
        // new RunCommand(
        // m_robotContainer.m_robotDrive::zeroHeading,
        // m_robotContainer.m_robotDrive));
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (m_robotContainer.m_operatorController.leftTrigger().getAsBoolean()) {
            m_robotContainer.m_rollers.runRollers(RollerMode.SUCK);
        } else if (m_robotContainer.m_operatorController.rightTrigger().getAsBoolean()) {
            m_robotContainer.m_rollers.runRollers(RollerMode.SHOOTLOW);
        } else if (m_robotContainer.m_operatorController.leftBumper().getAsBoolean()) {
            m_robotContainer.m_rollers.runRollers(RollerMode.SHOOTMID);
        } else if (m_robotContainer.m_operatorController.rightBumper().getAsBoolean()) {
            m_robotContainer.m_rollers.runRollers(RollerMode.SHOOTHIGH);
        } else {
            m_robotContainer.m_rollers.runRollers(RollerMode.STOP);
        }

        if (m_robotContainer.m_operatorController.povDown().getAsBoolean()) {
            m_robotContainer.m_pivot.setSetpoint(Rotation2d.fromDegrees(167.25));
            m_robotContainer.m_pivot.pivot();
        } else if (m_robotContainer.m_operatorController.povUp().getAsBoolean()) {
            m_robotContainer.m_pivot.setSetpoint(Rotation2d.fromDegrees(0.0));
            m_robotContainer.m_pivot.pivot();
        } else if (m_robotContainer.m_operatorController.povLeft().getAsBoolean()) {
            m_robotContainer.m_pivot.setSetpoint(Rotation2d.fromDegrees(90.0));
            m_robotContainer.m_pivot.pivot();
        } else if (m_robotContainer.m_operatorController.povRight().getAsBoolean()) {
            m_robotContainer.m_pivot.setSetpoint(Rotation2d.fromDegrees(160.0));
            m_robotContainer.m_pivot.pivot();
        } else {
            m_robotContainer.m_pivot.runAtPercent(m_robotContainer.m_operatorController.getRightY());
        }
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
}
