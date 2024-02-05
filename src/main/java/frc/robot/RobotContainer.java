// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathfindToTarget;
import frc.robot.commands.PositionWithAmp;
import frc.robot.commands.StrafeAndAimToAprilTag;
import frc.robot.commands.StrafeAndAimToSpeaker;
import frc.robot.commands.TeleopDrive;
import frc.robot.pathplanner.PathPlannerUtils;
import frc.robot.pathplanner.paths.PathPlannerPaths;
import frc.robot.subsystems.drive.DriveSubsystem;
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

    // The robot's subsystems
    public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision::pollLatestVisionMeasurement);

    // public final RollerSubsystem m_rollers = new RollerSubsystem();

    // public final PivotSubsystem m_pivot = new PivotSubsystem();

    final CommandCustomController m_driverController = new CommandCustomController(
            OIConstants.kDriverControllerPort);

    final CommandCustomController m_operatorController = new CommandCustomController(
            OIConstants.kOperatorControllerPort);

    // private final SendableChooser<Command> autoChooser;
    public final LoggedDashboardChooser<Command> m_autoChooser;

    public LoggedDashboardChooser<Command> m_pathChooser;

    public Map<String, Command> laneAssistCommands = new HashMap<>();
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

        // m_pivot.setDefaultCommand(new RunCommand(() -> {
        // m_pivot.runAtPercent(m_operatorController.getRightY());
        // }, m_pivot));

        // m_pivot.setDefaultCommand(m_pivot.getPivotCommand());

        // Build an auto chooser. You can make a default auto by passing in their name
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("New Auto"));
    }

    /**
     * Register the commands with PathPlanner
     */
    private void configureNamedCommands() {

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
                m_robotDrive),
                Set.of(m_robotDrive));

        laneAssistCommands.put("Speaker", speakerLaneAssistCommand);
        laneAssistCommands.put("Amp", ampLaneAssistCommand);

        // Arbitrary. This can be updated manually later.
        String defaultKey = "Speaker";

        m_laneAssistChooser = new LoggedDashboardChooser<>("Lane Assist", new SendableChooser<>());

        if (laneAssistCommands.size() != 0) {
            m_laneAssistChooser.addDefaultOption(defaultKey, laneAssistCommands.get(defaultKey));
            laneAssistCommands.keySet().forEach((String key) -> {
                m_laneAssistChooser.addOption(key, laneAssistCommands.get(key));
            });
        }

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
        m_driverController.rightBumper()
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setCross(),
                        m_robotDrive));

        m_driverController.povUp()
                .whileTrue(
                        Commands.deferredProxy(
                                () -> m_pathChooser.get()));

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
        m_driverController.leftBumper()
                .whileTrue(
                        Commands.defer(() -> new StrafeAndAimToAprilTag(
                                () -> -m_driverController.getLeftY(),
                                () -> -m_driverController.getLeftX(),
                                m_vision::getVisibleAprilTags,
                                3,
                                m_robotDrive),
                                Set.of(m_robotDrive)));
        // m_driverController.leftTrigger()
        // .whileTrue(
        // Commands.defer(() -> new StrafeAndAimToSpeaker(
        // () -> -m_driverController.getLeftY(),
        // () -> -m_driverController.getLeftX(),
        // m_vision::getVisibleAprilTags,
        // m_robotDrive),
        // Set.of(m_robotDrive)));
        // m_driverController.rightTrigger()
        // .whileTrue(
        // Commands.defer(() -> new PositionWithAmp(
        // () -> -m_driverController.getLeftX(),
        // m_vision::getVisibleAprilTags,
        // m_robotDrive),
        // Set.of(m_robotDrive)));
    }
}
