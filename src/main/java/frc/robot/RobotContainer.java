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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.AdvantageKitConstants.Mode;
import frc.robot.Constants.AmpTrapConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.bobot_state.AimingMode;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.AimAtNote;
import frc.robot.commands.PositionWithAmp;
import frc.robot.commands.PositionWithSpeaker;
import frc.robot.commands.PositionWithStageSingleClimb;
import frc.robot.commands.StrafeAndAimToSpeaker;
import frc.robot.commands.TeleopDrive;
import frc.robot.pathplanner.PathPlannerUtils;
import frc.robot.pathplanner.paths.PathPlannerPoses;
import frc.robot.subsystems.amptrap.AmpTrapSubsystem;
import frc.robot.subsystems.blinkin.BlinkinSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.apriltag.OffsetTags;
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

    final CommandCustomController m_programmerController = AdvantageKitConstants.getMode() == Mode.SIM
            ? new CommandCustomController(OIConstants.kProgrammerControllerPort)
            : null;

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

    public final AmpTrapSubsystem m_ampTrap = new AmpTrapSubsystem();

    public final ClimberSubsystem m_climber = new ClimberSubsystem();

    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

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
        m_pivot.setDefaultCommand(m_pivot.runTrapezoidProfileCommand());
        m_climber.setDefaultCommand(m_climber.pidCommand());
        m_elevator.setDefaultCommand(m_elevator.pidCommand());
        // Build an auto chooser. You can make a default auto by passing in their name
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureDriverBindings() {
        m_driverController.leftTrigger()
                .whileTrue(
                        m_intake.setVelocityThenStopCommand(IntakeConstants.kIntakeVelocity)
                                .alongWith(m_shooter
                                        .setVelocityFeederBeambreakCommand(ShooterConstants.kFeederIntakeVelocity)));

        m_driverController.rightTrigger()
                .and(m_shooter.beambreakIsObstructed().negate())
                .whileTrue(
                        new ParallelCommandGroup(
                                // Commands.defer(
                                // () -> new PathfindToTarget(
                                // m_vision::getClosestObject,
                                // m_robotDrive),
                                // Set.of(m_robotDrive)),
                                Commands.defer(
                                        () -> new AimAtNote(
                                                m_vision::getClosestObject,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                m_robotDrive),
                                        Set.of(m_robotDrive)),
                                m_intake.setVelocityThenStopCommand(IntakeConstants.kIntakeVelocity)
                                        .alongWith(m_shooter.setVelocityFeederBeambreakCommand(
                                                ShooterConstants.kFeederIntakeVelocity))));

        m_driverController.leftBumper()
                .whileTrue(Commands.deferredProxy(() -> m_laneAssistChooser.get().pathfindCommand()));
        m_driverController.rightBumper()
                .whileTrue(Commands.deferredProxy(() -> m_laneAssistChooser.get().aimingCommand()));
    }

    private void configureOperatorBindings() {
        m_operatorController.leftTrigger()
                .whileTrue(new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.shootAtSpeakerCommand()))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.leftTrigger().and(m_operatorController.rightTrigger())
                .whileTrue(new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.fireAtSpeakerCommand(ShooterConstants.kFeederShootVelocity)))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.rightTrigger()
                .onTrue(m_shooter.setVelocityFeederCommand(ShooterConstants.kFeederShootVelocity))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.leftY()
                .whileTrue(m_climber.runPercentOutputCommand(() -> -m_operatorController.getLeftY()))
                .onFalse(m_climber.setSetpointCurrentCommand());

        m_operatorController.rightY()
                .whileTrue(m_pivot.runPercentCommand(() -> -m_operatorController.getRightY() / 3.0))
                .onFalse(m_pivot.setEverythingCurrentCommand());

        // presets
        // 15 ft
        m_operatorController.povUp()
                .onTrue(
                        m_shooter.setVelocityShooterCommand(85.0, 75.0)
                                .alongWith(m_pivot.setGoalCommand(Rotation2d.fromDegrees(27.875))))
                .onFalse(m_shooter.stopCommand());
        m_operatorController.povDown()
                .onTrue(
                        m_shooter.setVelocityShooterCommand(47.8, 47.8)
                                .alongWith(m_pivot.setGoalCommand(Rotation2d.fromDegrees(42))))
                .onFalse(m_shooter.stopCommand());
        // up against the subwoofer
        // m_operatorController.povDown()
        // .whileTrue(m_pivot.runTrapezoidProfileCommand())
        // .onTru e(
        // m_shooter.setVelocityShooterCommand(60.0, 60.0)
        // .alongWith(m_pivot.setSetpointStateCommand(Rotation2d.fromDegrees(55))))
        // .onFalse(m_shooter.stopCommand());
        // 10ft shot
        m_operatorController.povLeft()
                .onTrue(m_shooter.setVelocityShooterCommand(85.0, 75.0)
                        .alongWith(m_pivot.setGoalCommand(Rotation2d.fromDegrees(36))))
                .onFalse(m_shooter.stopCommand());
        // 13ft shot
        m_operatorController.povRight()
                .onTrue(m_shooter.setVelocityShooterCommand(85.0, 75.0)
                        .alongWith(m_pivot.setGoalCommand(Rotation2d.fromDegrees(31.5))))
                .onFalse(m_shooter.stopCommand());

        // Fire the shooter, works with presets as well
        // m_operatorController.rightTrigger()
        // .whileTrue(m_shooter.setVelocityFeederCommand(ShooterConstants.kFeederShootVelocity));

        // Keep auto-aim active, but fire when ready.
        // m_operatorController
        // .a()
        // .and(m_operatorController.rightTrigger())
        // .whileTrue(
        // new ParallelCommandGroup(
        // m_pivot.pivotToSpeakerCommand(),
        // m_shooter.fireAtSpeakerCommand(ShooterConstants.kFeederShootVelocity)));

        // Move the Pivot before raising or lowering the Elevator
        m_operatorController.y()
                .and(m_elevator.elevatorIsAtTrap().negate())
                .onTrue(m_elevator.setSetpointCommand(ElevatorConstants.kMaxHeightInches));

        // Get the Pivot out of the way before lowering the Elevator
        m_operatorController.x()
                .onTrue(m_pivot.controlOutOfTheElevatorsWay()
                        .until(m_pivot.isBelowElevatorConflictTreshold())
                        .andThen(m_elevator.setSetpointCommand(
                                ElevatorConstants.kMinHeightInches)));

        // Move the Pivot out of the elevators way, then move the elevator to AMP score
        // mode, then move pivot to feed the AMP.
        m_operatorController.b()
                .and(m_elevator.elevatorIsAtAmp().negate())
                .onTrue(m_elevator.setSetpointCommand(ElevatorConstants.kAmpScoreHeightInches));

        // Assuming that the Operator set the setpoint, we move the pivot to fit
        // into the Amp/Trap mechanism.
        m_operatorController.b().and(m_operatorController.rightBumper())
                .and(m_elevator.elevatorIsAtAmp())
                .onTrue(m_pivot.setGoalToAmpScoringPosition());

        // Assuming that both the PivotAngle and the Elevator Height are right,
        // score into the AMP.
        m_operatorController.b().and(m_operatorController.rightTrigger())
                .and(m_elevator.elevatorIsAtAmp())
                .and(m_pivot.isNearAmpScoringAngle())
                .whileTrue(m_shooter.shootIntoAmpCommand())
                .onTrue(m_ampTrap.setVelocityCommand(AmpTrapConstants.kAmpSpeed))
                .onFalse(m_shooter.stopCommand().alongWith(m_ampTrap.stopCommand()));

        // Trap
        // Move the Pivot out of the elevators way, then move the elevator to Trap score
        // mode, then move pivot to feed the AMP.
        m_operatorController.y()
                .and(m_elevator.elevatorIsDown())
                .onTrue(m_elevator.setSetpointCommand(ElevatorConstants.kTrapScoreHeightInches));

        // Assuming that the Operator set the setpoint, we move the pivot to fit into
        // the Trap mechanism.
        m_operatorController.y().and(m_operatorController.rightBumper())
                .and(m_elevator.elevatorIsAtTrap())
                .onTrue(m_pivot.setGoalToTrapScoringPosition());

        // Assuming that both the PivotAngle and the Elevator Height are right,
        // score into the Trap.
        m_operatorController.y().and(m_operatorController.rightTrigger())
                .and(m_elevator.elevatorIsAtTrap())
                .and(m_pivot.isNearTrapScoringAngle())
                .whileTrue(m_shooter.shootIntoAmpCommand())
                .onTrue(m_ampTrap.setVelocityCommand(AmpTrapConstants.kTrapSpeed))
                .onFalse(m_shooter.stopCommand().alongWith(m_ampTrap.stopCommand()));

        // @TODO add controls for Trap, should look similar to the AMP scoring controls

        // m_pivot.movePivotOutOfTheElevatorsWay()
        // .until(m_pivot.pivotIsBelowElevatorMax())
        // .andThen(m_elevator.setSetpointCommand(ElevatorConstants.kAmpScoreHeightInches))
        // .until(m_elevator.elevatorIsUp())
        // .andThen(m_pivot.movePivotToAmpScoringPosition())

        // .whileTrue(m_pivot.runTrapezoidProfileCommand())
        // .onTrue(m_pivot.setSetpointStateCommand(PivotLocation.INITIAL.angle).andThen(Commands.none()));
    }

    private void configureProgrammerBindings() {
    }

    /**
     * Register the commands with PathPlanner
     */
    private void configureNamedCommands() {
        NamedCommands.registerCommand("RunIntakeReal",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                m_shooter.setVelocityFeederBeambreakCommand(ShooterConstants.kFeederIntakeVelocity),
                                m_intake.setVelocityCommand(IntakeConstants.kIntakeVelocity)),
                        m_intake.stopCommand()));

        // Run Interpolation in Parallel
        NamedCommands.registerCommand(
                "Interpolate",
                new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.shootAtSpeakerCommand()));

        NamedCommands.registerCommand(
                "InterpolateSetpoint",
                new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.shootAtSpeakerCommand()));

        NamedCommands.registerCommand(
                "AimAtSpeaker",
                Commands.defer(() -> new StrafeAndAimToSpeaker(
                        () -> 0,
                        () -> 0,
                        m_robotDrive),
                        Set.of(m_robotDrive)));

        NamedCommands.registerCommand("InterpolatePivot", m_pivot.controlGoalToSpeakerCommand());

        NamedCommands.registerCommand("InterpolatePivotSetpoint", m_pivot.controlGoalToSpeakerCommand());

        NamedCommands.registerCommand("RampShooter", m_shooter.shootAtSpeakerCommand());

        NamedCommands.registerCommand("PivotPID", m_pivot.runTrapezoidProfileCommand());

        NamedCommands.registerCommand(
                "Shoot",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(1),
                                m_shooter.fireAtSpeakerCommand(ShooterConstants.kFeederShootVelocity)),
                        m_shooter.stopFeederCommand()));

        NamedCommands.registerCommand(
                "FIRE!",
                new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_intake.setVelocityCommand(IntakeConstants.kIntakeVelocity),
                        m_shooter.fireAtSpeakerCommand(ShooterConstants.kFeederShootVelocity)));

        NamedCommands.registerCommand(
                "FireOne",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(1),
                                m_pivot.controlGoalToSpeakerCommand(),
                                m_intake.setVelocityCommand(IntakeConstants.kIntakeVelocity),
                                m_shooter.fireAtSpeakerCommand(ShooterConstants.kFeederShootVelocity)),
                        m_shooter.stopFeederCommand()));

        NamedCommands.registerCommand(
                "TargetNote",
                new InstantCommand(() -> BobotState.updateAimingMode(AimingMode.OBJECT_DETECTION)));

        NamedCommands.registerCommand(
                "TargetSpeaker",
                new InstantCommand(() -> BobotState.updateAimingMode(AimingMode.SPEAKER)));

        NamedCommands.registerCommand(
                "TargetRotation",
                new InstantCommand(() -> BobotState.updateAimingMode(AimingMode.NONE)));
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
        Command speakerStrafeAndAimCommand = new ParallelCommandGroup(
                Commands.defer(() -> new StrafeAndAimToSpeaker(
                        () -> -m_driverController.getLeftY(),
                        () -> -m_driverController.getLeftX(),
                        m_robotDrive),
                        Set.of(m_robotDrive)));

        Command speakerPosition10Command = new ParallelCommandGroup(
                Commands.defer(() -> new PositionWithSpeaker(
                        () -> -m_driverController.getLeftX(),
                        m_robotDrive,
                        OffsetTags.SPEAKER_10FT),
                        Set.of(m_robotDrive)));

        Command speakerPosition15Command = new ParallelCommandGroup(
                Commands.defer(() -> new PositionWithSpeaker(
                        () -> -m_driverController.getLeftX(),
                        m_robotDrive,
                        OffsetTags.SPEAKER_15FT),
                        Set.of(m_robotDrive)));

        Command ampCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftX(),
                m_robotDrive,
                OffsetTags.AMP),
                Set.of(m_robotDrive));

        Command otherAmpCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftY(),
                m_robotDrive,
                OffsetTags.OTHER_AMP),
                Set.of(m_robotDrive));

        Command stageHumanCommand = Commands.defer(() -> new SequentialCommandGroup(
                new PositionWithStageSingleClimb(
                        () -> -m_driverController.getLeftY(),
                        OffsetTags.STAGE_HUMAN,
                        m_robotDrive)),
                Set.of(m_robotDrive));

        Command stageAmpCommand = Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> -m_driverController.getLeftY(),
                OffsetTags.STAGE_AMP,
                m_robotDrive),
                Set.of(m_robotDrive));

        Command stageCenterCommand = Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> -m_driverController.getLeftY(),
                OffsetTags.STAGE_CENTER,
                m_robotDrive),
                Set.of(m_robotDrive));

        // Each command that we plan to use for 'Lane Assist' should be deferred
        // with their respective subsystems. Once they're deferred, we can then
        // proxy the deferred command to run while the button is held.
        m_laneAssistChooser = new LoggedDashboardChooser<>("LaneAssist", new SendableChooser<>());

        m_laneAssistCommands.put("Amp",
                new LaneAssist(OffsetTags.AMP.getDeferredCommand(), ampCommand));
        m_laneAssistCommands.put("Other Amp",
                new LaneAssist(OffsetTags.OTHER_AMP.getDeferredCommand(), otherAmpCommand));
        m_laneAssistCommands.put("Human Player",
                new LaneAssist(PathPlannerPoses.HUMAN_PLAYER.getDeferredCommand(),
                        new InstantCommand()));
        m_laneAssistCommands.put("Aim at Speaker",
                new LaneAssist(Commands.none(), speakerStrafeAndAimCommand));
        m_laneAssistCommands.put("Amp",
                new LaneAssist(OffsetTags.AMP.getDeferredCommand(), ampCommand));
        m_laneAssistCommands.put("Other Amp",
                new LaneAssist(OffsetTags.OTHER_AMP.getDeferredCommand(), otherAmpCommand));
        m_laneAssistCommands.put("Stage Center",
                new LaneAssist(OffsetTags.STAGE_CENTER.getDeferredCommand(), stageCenterCommand));
        m_laneAssistCommands.put("Stage Human",
                new LaneAssist(OffsetTags.STAGE_HUMAN.getDeferredCommand(), stageHumanCommand));
        m_laneAssistCommands.put("Stage Amp",
                new LaneAssist(OffsetTags.STAGE_AMP.getDeferredCommand(), stageAmpCommand));
        m_laneAssistCommands.put("Speaker (10 ft)",
                new LaneAssist(OffsetTags.SPEAKER_10FT.getDeferredCommand(), speakerPosition10Command));
        m_laneAssistCommands.put("Speaker (15 ft)",
                new LaneAssist(OffsetTags.SPEAKER_15FT.getDeferredCommand(), speakerPosition15Command));

        {
            String defaultLaneAssist = "Aim at Speaker";
            m_laneAssistChooser.addDefaultOption(defaultLaneAssist,
                    m_laneAssistCommands.get(defaultLaneAssist));
        }

        m_laneAssistCommands.forEach((String key, LaneAssist laneAssist) -> {
            m_laneAssistChooser.addOption(key, laneAssist);
        });

        // m_laneAssistChooser.addDefaultOption("Human Player",
        // m_laneAssistCommands.get("Human Player"));
    }
}
