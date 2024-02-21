// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathfindToTarget;
import frc.robot.commands.PositionWithAmp;
import frc.robot.commands.StrafeAndAimToSpeaker;
import frc.robot.commands.TeleopDrive;
import frc.robot.pathplanner.PathPlannerUtils;
import frc.robot.pathplanner.paths.PathPlannerPaths;
import frc.robot.subsystems.amptrap.AmpTrapSubsystem;
import frc.robot.subsystems.blinkin.BlinkinColors;
import frc.robot.subsystems.blinkin.BlinkinSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotLocation;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.utils.CommandCustomController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final Field2d field = new Field2d();

    public final VisionSubsystem m_vision = new VisionSubsystem();

    // ROBOT SUBSYSTEMS
    public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision::pollLatestVisionMeasurement);

    public final IntakeSubsystem m_intake = new IntakeSubsystem();

    public final PivotSubsystem m_pivot = new PivotSubsystem();

    public final ShooterSubsystem m_shooter = new ShooterSubsystem();

    public final AmpTrapSubsystem m_ampTrap = new AmpTrapSubsystem();

    public final ClimberSubsystem m_climber = new ClimberSubsystem();

    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    // public final MiscSubsystem m_misc = new MiscSubsystem();

    public final BlinkinSubsystem m_blinkin = new BlinkinSubsystem();

    final CommandCustomController m_driverController = new CommandCustomController(
            OIConstants.kDriverControllerPort);

    final CommandCustomController m_operatorController = new CommandCustomController(
            OIConstants.kOperatorControllerPort);

    // For tests only
    final CommandCustomController m_programmerController = new CommandCustomController(
            OIConstants.kProgrammerControllerPort);

    // private final SendableChooser<Command> autoChooser;
    public final LoggedDashboardChooser<Command> m_autoChooser;

    public LoggedDashboardChooser<Command> m_pathChooser;

    public final Map<String, Command> laneAssistCommands = new LinkedHashMap<>();

    public LoggedDashboardChooser<Command> m_laneAssistChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure PathPlanner logging with AdvantageKit
        PathPlannerUtils.configureLogging();

        // Configure the button bindings
        configureNamedCommands();
        configurePathChooser();
        configureButtonBindings();
        configureLaneAssistBindings();

        // Configure default commands
        // m_robotDrive.
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                TeleopDrive.asCommand(
                        m_robotDrive,
                        () -> -m_driverController.getLeftY(),
                        () -> -m_driverController.getLeftX(),
                        () -> -m_driverController.getRightX(),
                        true,
                        true));
        m_pivot.setDefaultCommand(m_pivot.pivotPIDCommand());
        m_climber.setDefaultCommand(m_climber.pidCommand());
        m_elevator.setDefaultCommand(m_elevator.pidCommand());
        // Build an auto chooser. You can make a default auto by passing in their name
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    /**
     * Register the commands with PathPlanner
     */
    private void configureNamedCommands() {
        NamedCommands.registerCommand("Shoot", new InstantCommand());
        NamedCommands.registerCommand("PickUpNote", new InstantCommand());
        NamedCommands.registerCommand("FindNote", new InstantCommand());
    }

    /**
     * Creates a state machine of paths and adds them all to it
     */
    private void configurePathChooser() {
        m_pathChooser = new LoggedDashboardChooser<>("Path Chooser", new SendableChooser<>());

        if (PathPlannerPaths.values().length != 0) {
            PathPlannerPaths defaultPath = PathPlannerPaths.values()[0];
            m_pathChooser.addDefaultOption(defaultPath.label, defaultPath.getCommand());

            for (PathPlannerPaths path : PathPlannerPaths.values()) {
                m_pathChooser.addOption(path.label, path.getCommand());
            }
        }
    }

    /**
     * Configure all commands that are used for 'Lane Assist'.
     *
     * <p>
     * The MVP is using a HashMap with display names for keys and the Command
     * for the value.
     * </p>
     *
     * <p>
     * We may be able to make this an Enum, but that would require
     * making most of the codebase static, which can have unintended side effects.
     * </p>
     */
    private void configureLaneAssistBindings() {
        Command speakerLaneAssistCommand = Commands.defer(() -> new StrafeAndAimToSpeaker(
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                m_vision::getVisibleAprilTags,
                m_robotDrive),
                Set.of(m_robotDrive));

        Command ampLaneAssistCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftX(),
                m_vision::getVisibleAprilTags,
                m_robotDrive,
                false),
                Set.of(m_robotDrive));

        Command coopertitionLaneAssistCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftX(),
                m_vision::getVisibleAprilTags,
                m_robotDrive,
                true),
                Set.of(m_robotDrive));

        String defaultKey = "Speaker Center";

        laneAssistCommands.put("Speaker Center", speakerLaneAssistCommand);
        laneAssistCommands.put("Speaker Left", speakerLaneAssistCommand);
        laneAssistCommands.put("Speaker Right", speakerLaneAssistCommand);

        laneAssistCommands.put("Amp", ampLaneAssistCommand);
        laneAssistCommands.put("Other Amp", coopertitionLaneAssistCommand);

        m_laneAssistChooser = new LoggedDashboardChooser<>("LaneAssist", new SendableChooser<>());
        laneAssistCommands.forEach((String key, Command command) -> {
            m_laneAssistChooser.addOption(key, command);
        });

        m_laneAssistChooser.addDefaultOption(defaultKey,
                laneAssistCommands.get(defaultKey));

        // Each command that we plan to use for 'Lane Assist' should be deferred
        // with their respective subsystems. Once they're deferred, we can then
        // proxy the deferred command to run while the button is held.
        m_driverController.rightTrigger()
                .whileTrue(Commands.deferredProxy(() -> m_laneAssistChooser.get()));
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
        SmartDashboard.putData("Run Chosen Path", Commands.deferredProxy(
                () -> m_pathChooser.get()));

        m_driverController
                .rightBumper()
                .whileTrue(
                        Commands.defer(
                                () -> new PathfindToTarget(
                                        m_vision::getClosestObject,
                                        m_robotDrive),
                                Set.of(m_robotDrive)));
        m_driverController.povUp()
                .whileTrue(
                        Commands.deferredProxy(
                                () -> m_pathChooser.get()));
        m_driverController.x()
                .onTrue(m_intake.setVelocityCommand(20))
                .onFalse(m_intake.stopCommand());
        m_driverController.y()
                .onTrue(m_intake.setVelocityCommand(50))
                .onFalse(m_intake.stopCommand());
        m_driverController.a()
                .and(m_shooter.beambreakIsActivated().negate())
                .onTrue(m_intake.setVelocityCommand(20))
                .onFalse(m_intake.stopCommand());

        m_driverController.povUp().onTrue(m_blinkin.setColorCommand(BlinkinColors.SOLID_RED));
        m_driverController.povRight().onTrue(m_blinkin.setColorCommand(BlinkinColors.SOLID_GREEN));
        m_driverController.povDown().onTrue(m_blinkin.setColorCommand(BlinkinColors.SOLID_BLUE));
        m_driverController.povLeft().onTrue(m_blinkin.setColorCommand(BlinkinColors.SOLID_YELLOW));

        // 60 rps shot, 10 feet out, 40 degrees shooter angle ()
        m_operatorController.x() /* 65rps 10ft 36degrees */
                .onTrue(m_shooter.setVelocityShooterCommand(65.0, 65.0))
                .onFalse(m_shooter.stopCommand());
        m_operatorController.y() /* 10rps amp */
                .onTrue(m_shooter.setVelocityShooterCommand(10.0, 10.0))
                .onFalse(m_shooter.stopCommand());
        m_operatorController.a() /* 21ft shot and preferably use for other distances */
                .onTrue(m_shooter.setVelocityShooterCommand(85.0, 70.0))
                .onFalse(m_shooter.stopCommand());
        m_operatorController.b()
                .onTrue(m_shooter.setVelocityShooterCommand(60.0, 60.0))
                .onFalse(m_shooter.stopCommand());
        m_operatorController.rightY()
                .whileTrue(m_pivot.runPercentCommand(() -> -m_operatorController.getRightY() / 2.0))
                .onFalse(m_pivot.setSetpointCurrentCommand());
        m_operatorController.povLeft().onTrue(m_pivot.setSetpointCommand(PivotLocation.k85.angle));
        m_operatorController.povUp().onTrue(m_pivot.setSetpointCommand(PivotLocation.k55.angle));
        m_operatorController.povRight().onTrue(m_pivot.setSetpointCommand(PivotLocation.k36.angle));
        m_operatorController.povDown().onTrue(m_pivot.setSetpointCommand(PivotLocation.k26.angle));

        // TEST CONTROLLER
        m_programmerController.x()
                .whileTrue(m_intake.setVelocityCommand(20.0))
                .onFalse(m_intake.stopCommand());
        m_programmerController.x()
                .and(m_intake.beambreakIsActivated())
                .whileTrue(m_shooter.setVelocityFeederCommand(20.0))
                .whileFalse(m_shooter.stopFeederCommand());
        m_programmerController.y()
                .onTrue(m_climber.setSetpointCommand(100.0));
        m_programmerController.a()
                .onTrue(m_climber.setSetpointCommand(0.0));
        m_programmerController.rightY()
                .whileTrue(m_climber.runSetpointController(() -> -m_programmerController.getRightY()))
                .onFalse(m_climber.setSetpointCurrentCommand());

        m_programmerController.povUp()
                .onTrue(m_intake.toggleBeamBrakeActivatedCommand());
    }
}
