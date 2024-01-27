// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.utils.CommandCustomController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final Field2d field = new Field2d();

    // The robot's subsystems
    // public final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // public final RollerSubsystem m_rollers = new RollerSubsystem();

    // public final PivotSubsystem m_pivot = new PivotSubsystem();

    public final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // public final MiscSubsystem m_misc = new MiscSubsystem();

    final CommandCustomController m_driverController = new CommandCustomController(OIConstants.kDriverControllerPort);

    final CommandCustomController m_operatorController = new CommandCustomController(
            OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // m_robotDrive.setDefaultCommand(
        // // The left stick controls translation of the robot.
        // // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        // () -> m_robotDrive.drive(
        // -m_driverController.getLeftY(),
        // -m_driverController.getLeftX(),
        // -m_driverController.getRightX(),
        // true, true),
        // m_robotDrive));

        // m_pivot.setDefaultCommand(new RunCommand(() -> {
        // m_pivot.runAtPercent(m_operatorController.getRightY());
        // }, m_pivot));

        // m_pivot.setDefaultCommand(m_pivot.getPivotCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // m_driverController.rightBumper()
        // .whileTrue(new RunCommand(
        // () -> m_robotDrive.setCross(),
        // m_robotDrive));

        m_driverController.b()
                .onTrue(m_shooter.setVelocityCommand(25.0, 25.0))
                .onFalse(m_shooter.stopCommand());
        m_driverController.y()
                .onTrue(m_shooter.setVelocityCommand(10.0, 10.0))
                .onFalse(m_shooter.stopCommand());

        // 60 rps shot, 10 feet out, 40 degrees shooter angle ()
        m_driverController.x()
                .onTrue(m_shooter.setVelocityCommand(65.0, 65.0))
                .onFalse(m_shooter.stopCommand());

        //
        m_driverController.a()
                .onTrue(m_shooter.setVelocityCommand(70.0, 50.0))
                .onFalse(m_shooter.stopCommand());

        // m_driverController.rightBumper()
        // .onTrue(m_misc.setVelocityCommand(20.0))
        // .onFalse(m_misc.stopCommand());

    }

}
