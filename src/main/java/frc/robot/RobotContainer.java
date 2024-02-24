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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.PathfindToTarget;
import frc.robot.commands.PositionWithAmp;
import frc.robot.commands.PositionWithSpeaker;
import frc.robot.commands.PositionWithStageSingleClimb;
import frc.robot.commands.StrafeAndAimToSpeaker;
import frc.robot.commands.TeleopDrive;
import frc.robot.pathplanner.PathPlannerUtils;
import frc.robot.pathplanner.paths.PathPlannerPoses;
import frc.robot.subsystems.blinkin.BlinkinColors;
import frc.robot.subsystems.blinkin.BlinkinSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotLocation;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.apriltag.StageTags;
import frc.utils.CommandCustomController;
import frc.utils.LaneAssist;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    final CommandCustomController m_driverController = new CommandCustomController(
            OIConstants.kDriverControllerPort);

    final CommandCustomController m_operatorController = new CommandCustomController(
            OIConstants.kOperatorControllerPort);

    // For tests only
    // final CommandCustomController m_programmerController = new
    // CommandCustomController(
    // OIConstants.kProgrammerControllerPort);

    public final Field2d field = new Field2d();

    public final VisionSubsystem m_vision = new VisionSubsystem();

    public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision::pollLatestVisionMeasurement);

    public final IntakeSubsystem m_intake = new IntakeSubsystem();

    public final PivotSubsystem m_pivot = new PivotSubsystem();

    public final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // public final AmpTrapSubsystem m_ampTrap = new AmpTrapSubsystem();

    // public final ClimberSubsystem m_climber = new ClimberSubsystem();

    // public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    public final BlinkinSubsystem m_blinkin = new BlinkinSubsystem();

    // private final SendableChooser<Command> autoChooser;
    public final LoggedDashboardChooser<Command> m_autoChooser;

    public final Map<String, LaneAssist> m_laneAssistCommands = new LinkedHashMap<>();

    public LoggedDashboardChooser<LaneAssist> m_laneAssistChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        new BobotState(); // this is a no-op but is required for it to be registered as a VirtualSubsystem

        // Configure PathPlanner logging with AdvantageKit
        PathPlannerUtils.configureLogging();

        // Configure the button bindings
        configureNamedCommands();
        // configureButtonBindings();
        configureDriverBindings();
        configureOperatorBindings();
        configureProgrammerBindings();
        configureLaneChooser();

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
        // m_climber.setDefaultCommand(m_climber.pidCommand());
        // m_elevator.setDefaultCommand(m_elevator.pidCommand());
        // Build an auto chooser. You can make a default auto by passing in their name
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureDriverBindings() {
        m_driverController.leftTrigger()
                .whileTrue(
                        m_intake.setVelocityThenStopCommand(20)
                                .alongWith(m_shooter.setVelocityFeederBeambreakCommand(20)));

        m_driverController.rightTrigger()
                .whileTrue(
                        Commands.defer(
                                () -> new PathfindToTarget(
                                        m_vision::getClosestObject,
                                        m_robotDrive),
                                Set.of(m_robotDrive))
                                .alongWith(m_intake.setVelocityCommand(20)));

        m_driverController.leftBumper()
                .whileTrue(Commands.deferredProxy(() -> m_laneAssistChooser.get().pathfindCommand()));
        m_driverController.rightBumper()
                .whileTrue(Commands.deferredProxy(() -> m_laneAssistChooser.get().aimingCommand()));
    }

    private void configureOperatorBindings() {
        m_operatorController.leftTrigger()
                .whileTrue(m_shooter.setVelocityFeederCommand(50));

        m_operatorController.rightTrigger()
                .onTrue(m_shooter.setVelocityShooterCommand(85.0, 70.0))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.rightY()
                .whileTrue(m_pivot.runPercentCommand(() -> -m_operatorController.getRightY() / 3.0))
                .onFalse(m_pivot.setSetpointCurrentCommand());

        // presets
        // 10 ft
        m_operatorController.povUp()
                .onTrue(
                        m_shooter.setVelocityShooterCommand(65.0, 65.0)
                                .alongWith(m_pivot.setSetpointCommand(PivotLocation.k36.angle)))
                .onFalse(m_shooter.stopCommand());
        // 15 ft
        m_operatorController.povRight()
                .onTrue(
                        m_shooter.setVelocityShooterCommand(65.0, 65.0)
                                .alongWith(m_pivot.setSetpointCommand(Rotation2d.fromDegrees(31))))
                .onFalse(m_shooter.stopCommand());
        // 21 ft
        m_operatorController.povDown()
                .onTrue(
                        m_shooter.setVelocityShooterCommand(85.0, 75.0)
                                .alongWith(m_pivot.setSetpointCommand(Rotation2d.fromDegrees(26))))
                .onFalse(m_shooter.stopCommand());

    }

    private void configureProgrammerBindings() {
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
    private void configureLaneChooser() {
        Command speakerStrafeAndAimCommand = Commands.defer(() -> new StrafeAndAimToSpeaker(
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                m_vision::getVisibleAprilTags,
                m_robotDrive),
                Set.of(m_robotDrive));

        Command speakerPosition10Command = new ParallelCommandGroup(
                m_pivot.setSetpointCommand(PivotLocation.k36.angle),
                Commands.defer(() -> new PositionWithSpeaker(
                        () -> -m_driverController.getLeftX(),
                        m_vision::getVisibleAprilTags,
                        m_robotDrive,
                        StageTags.SPEAKER_10FT),
                        Set.of(m_robotDrive)));

        Command speakerPosition15Command = new ParallelCommandGroup(
                m_pivot.setSetpointCommand(PivotLocation.INITIAL.angle),
                Commands.defer(() -> new PositionWithSpeaker(
                        () -> -m_driverController.getLeftX(),
                        m_vision::getVisibleAprilTags,
                        m_robotDrive,
                        StageTags.SPEAKER_15FT),
                        Set.of(m_robotDrive)));

        Command ampCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftX(),
                m_vision::getVisibleAprilTags,
                m_robotDrive,
                false),
                Set.of(m_robotDrive));

        Command otherAmpCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftY(),
                m_vision::getVisibleAprilTags,
                m_robotDrive,
                true),
                Set.of(m_robotDrive));

        Command stageHumanCommand = Commands.defer(() -> new SequentialCommandGroup(
                new PositionWithStageSingleClimb(
                        () -> -m_driverController.getLeftY(),
                        m_vision::getVisibleAprilTags,
                        StageTags.HUMAN,
                        m_robotDrive)),
                Set.of(m_robotDrive));

        Command stageAmpCommand = Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> -m_driverController.getLeftY(),
                m_vision::getVisibleAprilTags,
                StageTags.AMP,
                m_robotDrive),
                Set.of(m_robotDrive));

        Command stageCenterCommand = Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> -m_driverController.getLeftY(),
                m_vision::getVisibleAprilTags,
                StageTags.CENTER,
                m_robotDrive),
                Set.of(m_robotDrive));

        // Each command that we plan to use for 'Lane Assist' should be deferred
        // with their respective subsystems. Once they're deferred, we can then
        // proxy the deferred command to run while the button is held.
        m_laneAssistChooser = new LoggedDashboardChooser<>("LaneAssist", new SendableChooser<>());

        m_laneAssistCommands.put("Amp",
                new LaneAssist(PathPlannerPoses.AMP.getDeferredCommand(), ampCommand));
        m_laneAssistCommands.put("Other Amp",
                new LaneAssist(PathPlannerPoses.OTHER_AMP.getDeferredCommand(), otherAmpCommand));
        m_laneAssistCommands.put("Human Player",
                new LaneAssist(PathPlannerPoses.HUMAN_PLAYER.getDeferredCommand(),
                        new InstantCommand()));
        m_laneAssistCommands.put("Speaker Left",
                new LaneAssist(PathPlannerPoses.SPEAKER_LEFT.getDeferredCommand(),
                        speakerStrafeAndAimCommand));
        m_laneAssistCommands.put("Speaker Center",
                new LaneAssist(PathPlannerPoses.SPEAKER_CENTER.getDeferredCommand(),
                        speakerStrafeAndAimCommand));
        m_laneAssistCommands.put("Speaker Right",
                new LaneAssist(PathPlannerPoses.SPEAKER_RIGHT.getDeferredCommand(),
                        speakerStrafeAndAimCommand));
        m_laneAssistCommands.put("Amp",
                new LaneAssist(PathPlannerPoses.AMP.getDeferredCommand(), ampCommand));
        m_laneAssistCommands.put("Other Amp",
                new LaneAssist(PathPlannerPoses.OTHER_AMP.getDeferredCommand(), otherAmpCommand));
        m_laneAssistCommands.put("Stage Center",
                new LaneAssist(StageTags.CENTER.getDeferredCommand(), stageCenterCommand));
        m_laneAssistCommands.put("Stage Human",
                new LaneAssist(StageTags.HUMAN.getDeferredCommand(), stageHumanCommand));
        m_laneAssistCommands.put("Stage Amp",
                new LaneAssist(StageTags.AMP.getDeferredCommand(), stageAmpCommand));
        m_laneAssistCommands.put("Speaker (10 ft)",
                new LaneAssist(StageTags.SPEAKER_10FT.getDeferredCommand(), speakerPosition10Command));
        m_laneAssistCommands.put("Speaker (15 ft)",
                new LaneAssist(StageTags.SPEAKER_15FT.getDeferredCommand(), speakerPosition15Command));

        {
            String defaultLaneAssist = "Amp";
            m_laneAssistChooser.addDefaultOption(defaultLaneAssist,
                    m_laneAssistCommands.get(defaultLaneAssist));
        }

        m_laneAssistCommands.forEach((String key, LaneAssist laneAssist) -> {
            m_laneAssistChooser.addOption(key, laneAssist);
        });

        m_laneAssistChooser.addDefaultOption("Human Player", m_laneAssistCommands.get("Human Player"));
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

        // 60 rps shot, 10 feet out, 40 degrees shooter angle ()
        // m_driverController.x()
        // .onTrue(m_shooter.setVelocityShooterCommand(65.0, 65.0))
        // .onFalse(m_shooter.stopCommand());

        //
        m_driverController.a()
                .onTrue(m_shooter.setVelocityShooterCommand(85.0, 70.0))
                .onFalse(m_shooter.stopCommand());

        // m_driverController.rightBumper()
        // .onTrue(m_misc.setVelocityCommand(20.0))
        // .onFalse(m_misc.stopCommand());

        m_driverController.rightBumper()
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setCross(),
                        m_robotDrive));

        m_operatorController.rightY()
                .whileTrue(m_pivot.runPercentCommand(() -> -m_operatorController.getRightY() / 2.0))
                .onFalse(m_pivot.setSetpointCurrentCommand());
        // m_operatorController.povUp().onTrue(m_pivot.setSetpointCommand(PivotLocation.k0.angle));
        // m_operatorController.povDown().onTrue(m_pivot.setSetpointCommand(PivotLocation.k45.angle));
        // m_operatorController.povLeft().onTrue(m_pivot.setSetpointCommand(PivotLocation.k90.angle));
        m_driverController.a().whileTrue(
                new ParallelCommandGroup(
                        m_pivot.pivotToSpeakerCommand(),
                        m_shooter.shootAtSpeakerCommand()));
        // m_driverController.leftTrigger()
        // .whileTrue(
        // Commands.deferredProxy(
        // () -> m_pathChooser.get()));

        m_driverController
                .rightBumper()
                .whileTrue(
                        Commands.defer(
                                () -> new PathfindToTarget(
                                        m_vision::getClosestObject,
                                        m_robotDrive),
                                Set.of(m_robotDrive)));
        // m_driverController.leftBumper()
        // .whileTrue(
        // Commands.defer(() -> new StrafeAndAimToAprilTag(
        // () -> -m_driverController.getLeftY(),
        // () -> -m_driverController.getLeftX(),
        // m_vision::getVisibleAprilTags,
        // 3,
        // m_robotDrive),
        // Set.of(m_robotDrive)));
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
        m_driverController.b().onTrue(m_intake.setBeambreakObstructedCommand(true));
        m_driverController.a().onTrue(m_intake.setBeambreakObstructedCommand(false));
        m_driverController.y()
                .onTrue(m_intake.setVelocityCommand(50))
                .onFalse(m_intake.stopCommand());
        m_driverController.x()
                .whileTrue(m_intake.setVelocityBeambreakCommand(20));

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
        m_operatorController.povRight().onTrue(m_pivot.setSetpointCommand(PivotLocation.k36.angle));
        m_operatorController.povDown().onTrue(m_pivot.setSetpointCommand(PivotLocation.INITIAL.angle));

        // TEST CONTROLLER
        // m_programmerController.x()
        // .whileTrue(m_intake.setVelocityCommand(20.0))
        // .onFalse(m_intake.stopCommand());
        // m_programmerController.x()
        // .and(m_intake.beambreakIsObstructed())
        // .whileTrue(m_shooter.setVelocityFeederCommand(20.0))
        // .whileFalse(m_shooter.stopFeederCommand());
        // m_programmerController.y()
        // .onTrue(m_climber.setSetpointCommand(100.0));
        // m_programmerController.a()
        // .onTrue(m_climber.setSetpointCommand(0.0));
        // m_programmerController.rightY()
        // .whileTrue(m_climber.runSetpointController(() ->
        // -m_programmerController.getRightY()))
        // .onFalse(m_climber.setSetpointCurrentCommand());

        // m_programmerController.povUp()
        // .onTrue(m_intake.toggleBeambreakObstructedCommand());
    }
}
