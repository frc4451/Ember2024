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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.bobot_state.AimingMode;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.AimAtNote;
import frc.robot.commands.PositionWithAmp;
import frc.robot.commands.PositionWithSpeaker;
import frc.robot.commands.StrafeAndAimToSpeaker;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.pathplanner.PathPlannerUtils;
import frc.robot.pathplanner.paths.PathPlannerPoses;
import frc.robot.subsystems.blinkin.BlinkinState;
import frc.robot.subsystems.blinkin.BlinkinSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotLocation;
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

    public final FeederSubsystem m_feeder = new FeederSubsystem();

    // public final AmpTrapSubsystem m_ampTrap = new AmpTrapSubsystem();

    public final ClimberSubsystem m_climber = new ClimberSubsystem();

    // public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    public final DriverAutomationFactory m_automation = new DriverAutomationFactory(
            m_driverController,
            m_operatorController,
            m_robotDrive,
            m_pivot,
            m_shooter,
            m_feeder);

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
        configureLights();
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
        // m_elevator.setDefaultCommand(m_elevator.pidCommand());
        // Build an auto chooser. You can make a default auto by passing in their name
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

        m_autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                // Since we don't anything in the drive subsystem to easily orient the
                // swerve modules correctly, we just run the command for a second to set them.
                // After that we can actually run the characterization.
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(1.0),
                                new WheelRadiusCharacterization(
                                        m_robotDrive,
                                        WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE)),
                        new WheelRadiusCharacterization(
                                m_robotDrive,
                                WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE)));
    }

    private void configureDriverBindings() {
        m_driverController.povDown()
                .whileTrue(
                        m_intake.setPercentOutputThenStopCommand(
                                IntakeConstants.kIntakeReversePercent)
                                .alongWith(m_feeder
                                        .setVelocityCommand(
                                                FeederConstants.kReverseVelocity)))
                .onFalse(m_feeder.stopCommand());

        m_driverController.leftTrigger()
                .whileTrue(
                        m_intake.setPercentOutputThenStopCommand(IntakeConstants.kIntakePercent)
                                .alongWith(m_feeder.setVelocityBeambreakCommand(
                                        FeederConstants.kIntakeVelocity)));

        m_driverController.rightTrigger()
                .and(m_feeder.beambreakIsObstructed().negate())
                .whileTrue(
                        new ParallelCommandGroup(
                                Commands.defer(
                                        () -> new AimAtNote(
                                                m_vision::getClosestObject,
                                                () -> -m_driverController
                                                        .getLeftY(),
                                                () -> -m_driverController
                                                        .getLeftX(),
                                                () -> -m_driverController
                                                        .getRightX(),
                                                m_robotDrive),
                                        Set.of(m_robotDrive)),
                                m_intake.setPercentOutputThenStopCommand(
                                        IntakeConstants.kIntakePercent)
                                        .alongWith(m_feeder
                                                .setVelocityBeambreakCommand(
                                                        FeederConstants.kIntakeVelocity))));

        // m_driverController.leftBumper()
        // .whileTrue(Commands.deferredProxy(() ->
        // m_laneAssistChooser.get().pathfindCommand()));
        // m_driverController.rightBumper()
        // .whileTrue(Commands.deferredProxy(() ->
        // m_laneAssistChooser.get().aimingCommand()));

        m_driverController.a() // .and(m_driverController.leftBumper().negate())
                .whileTrue(m_automation.ampPath());

        m_driverController.a().and(m_driverController.leftBumper())
                .whileTrue(m_automation.ampAssist());

        m_driverController.b() // .and(m_driverController.leftBumper().negate())
                .whileTrue(m_automation.stageRightPath());

        m_driverController.b().and(m_driverController.leftBumper())
                .whileTrue(m_automation.stageRightAssist());

        m_driverController.y() // .and(m_driverController.leftBumper().negate())
                .whileTrue(m_automation.stageCenterPath());

        m_driverController.y().and(m_driverController.leftBumper())
                .whileTrue(m_automation.stageCenterAssist());

        m_driverController.x() // .and(m_driverController.leftBumper().negate())
                .whileTrue(m_automation.stageLeftPath());

        m_driverController.x().and(m_driverController.leftBumper())
                .whileTrue(m_automation.stageLeftAssist());

        m_driverController.rightBumper()
                .whileTrue(m_automation.aimAtSpeakerAssist());

        m_driverController.leftBumper()
                .and(m_driverController.a().negate())
                .and(m_driverController.b().negate())
                .and(m_driverController.x().negate())
                .and(m_driverController.y().negate())
                .whileTrue(m_automation.strafeAndAimToAmpFeed());

        m_driverController.start()
                .whileTrue(m_automation.humanPlayerStationPath());

    }

    private void configureOperatorBindings() {
        m_operatorController.leftTrigger()
                .whileTrue(new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.rampUpSpeedToSpeakerCommand()))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.leftBumper()
                .whileTrue(new ParallelCommandGroup(
                        m_pivot.controlGoalToFloorCommand(),
                        m_shooter.rampUpSpeedToFloorCommand()))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.rightTrigger().and(m_operatorController.x().negate())
                .onTrue(m_feeder.setVelocityCommand(FeederConstants.kShootVelocity))
                .onFalse(m_feeder.stopCommand());

        m_operatorController.leftY()
                .whileTrue(m_climber.runPercentOutputCommand(() -> -m_operatorController.getLeftY()))
                .onFalse(m_climber.setSetpointCurrentCommand());

        m_operatorController.rightY()
                .whileTrue(m_pivot.runPercentCommand(() -> -m_operatorController.getRightY() / 3.0))
                .onFalse(m_pivot.setEverythingCurrentCommand());

        // Allred and Owen pooped
        m_operatorController.b()
                .whileTrue(new ParallelCommandGroup(
                        m_pivot.controlGoalToPoopCommand(),
                        m_shooter.rampUpSpeedToPoopCommand()))
                .onFalse(m_shooter.stopCommand());

        // m_operatorController.x()
        // .whileTrue(m_pivot.controlGoalToAmpShotCommand())
        // .onTrue(m_shooter.shootIntoAmpCommand())
        // .onFalse(m_shooter.stopCommand());

        // m_operatorController.x().and(m_operatorController.rightTrigger())
        // .whileTrue(m_feeder.feedIntoAmpCommand())
        // .onFalse(m_feeder.stopCommand());

        Rotation2d shootAngle = PivotLocation.kNewAmpShot.angle.minus(Rotation2d.fromDegrees(10));
        Rotation2d finalAngle = PivotLocation.kNewAmpShot.angle;

        m_operatorController.x()
                .onTrue(new ParallelCommandGroup(
                        m_pivot.setGoalCommand(shootAngle),
                        m_shooter.shootIntoAmpCommand()))
                .onFalse(m_shooter.stopCommand());

        m_operatorController.x().and(m_operatorController.rightTrigger())
                .onTrue(Commands.parallel(
                        m_pivot.setGoalCommand(finalAngle),
                        m_feeder.feedIntoAmpCommand()))
                .onFalse(m_feeder.stopCommand());

        m_operatorController.a()
                .onTrue(m_climber.setSetpointCommand(ClimberConstants.kMaxHeightInches));

        // m_operatorController.x()
        // .onTrue(m_automation.ampShot());

        m_operatorController.povUp()
                .whileTrue(new ParallelCommandGroup(
                        m_feeder.feedIntoAmpCommand(),
                        m_shooter.shootIntoAmpCommand()))
                .onFalse(new ParallelCommandGroup(
                        m_shooter.stopCommand(),
                        m_feeder.stopCommand()));

        // presets
        // Subwoofer shot
        // This could be made into a singular command sequence.
        m_operatorController.povDown()
                .onTrue(new ParallelCommandGroup(
                        m_shooter.setVelocityCommand(
                                ShooterConstants.kLeftShooterSpeed,
                                ShooterConstants.kRightShooterSpeed),
                        m_pivot.setGoalCommand(Rotation2d.fromDegrees(55.0))))
                .onFalse(new ParallelCommandGroup(m_shooter.stopShooterCommand()));

        // m_operatorController.povDown()
        // .and(m_elevator.elevatorIsAtSubwooferShot().negate())
        // .onTrue(new ParallelCommandGroup(
        // m_elevator.setSetpointCommand(
        // ElevatorConstants.kSubwooferShotHeightInches),
        // m_shooter.setVelocityCommand(
        // 60.0,
        // 50.0)));
        // m_operatorController.povDown()
        // .and(m_elevator.elevatorIsAtSubwooferShot())
        // .onTrue(new ParallelCommandGroup(
        // m_pivot.setGoalCommand(PivotLocation.kSubwooferScoringPosition.angle),
        // m_shooter.setVelocityCommand(
        // 60.0,
        // 50.0)))
        // .onFalse(m_shooter.stopCommand());

        // 15 ft
        // m_operatorController.povUp()
        // .onTrue(
        // m_shooter.setVelocityCommand(ShooterConstants.kLeftShooterSpeed,
        // ShooterConstants.kRightShooterSpeed)
        // .alongWith(m_pivot.setGoalCommand(
        // Rotation2d.fromDegrees(26.9))))
        // .onFalse(m_shooter.stopCommand());

        // 10ft shot
        // 10ft shot
        m_operatorController.povLeft()
                .onTrue(m_shooter
                        .setVelocityCommand(57.5, 57.5)
                        .alongWith(m_pivot.setGoalCommand(Rotation2d.fromDegrees(45))))
                .onFalse(m_shooter.stopCommand());
        // 13ft shot
        m_operatorController.povRight()
                .onTrue(m_shooter
                        .setVelocityCommand(ShooterConstants.kLeftShooterSpeed,
                                ShooterConstants.kRightShooterSpeed)
                        .alongWith(m_pivot.setGoalCommand(Rotation2d.fromDegrees(28.5))))
                .onFalse(m_shooter.stopCommand());

        // Fire the shooter, works with presets as well
        // m_operatorController.rightTrigger()
        // .whileTrue(m_shooter.setVelocityFeederCommand(FeederConstants.kFeederShootVelocity));

        // Keep auto-aim active, but fire when ready.
        // m_operatorController
        // .a()
        // .and(m_operatorController.rightTrigger())
        // .whileTrue(
        // new ParallelCommandGroup(
        // m_pivot.pivotToSpeakerCommand(),
        // m_shooter.fireAtSpeakerCommand(FeederConstants.kFeederShootVelocity)));

        // // Get the Pivot out of the way before lowering the Elevator
        // m_operatorController.x()
        // .onTrue(m_pivot.controlOutOfTheElevatorsWay()
        // .until(m_pivot.isBelowElevatorConflictTreshold())
        // .andThen(m_elevator.setSetpointCommand(
        // ElevatorConstants.kMinHeightInches)));

        // // AMP Scoring
        // m_operatorController.b()
        // .and(m_elevator.elevatorIsAtAmp().negate())
        // .onTrue(m_elevator.setSetpointCommand(ElevatorConstants.kAmpScoreHeightInches));

        // m_operatorController.b()
        // .and(m_elevator.elevatorIsAtAmp())
        // .onTrue(m_pivot.setGoalCommand(PivotLocation.kAmpScoringPosition.angle));

        // // Assuming that both the PivotAngle and the Elevator Height are right,
        // // score into the Trap.
        // m_operatorController.b().and(m_operatorController.rightTrigger())
        // .and(m_elevator.elevatorIsAtAmp())
        // .and(m_pivot.isNearAmpScoringAngle())
        // .onTrue(new ParallelCommandGroup(
        // m_ampTrap.setVelocityCommand(AmpTrapConstants.kAmpSpeed),
        // m_shooter.shootIntoAmpCommand()))
        // .onFalse(new ParallelCommandGroup(
        // m_ampTrap.stopCommand(),
        // m_shooter.stopCommand()));

        // // Trap
        // m_operatorController.y()
        // .and(m_elevator.elevatorIsAtTrap().negate())
        // .onTrue(m_elevator.setSetpointCommand(ElevatorConstants.kTrapScoreHeightInches));

        // m_operatorController.y()
        // .and(m_elevator.elevatorIsAtTrap())
        // .onTrue(m_pivot.setGoalCommand(PivotLocation.kTrapScoringPosition.angle));

        // // Assuming that both the PivotAngle and the Elevator Height are right,
        // // score into the Trap.
        // m_operatorController.y().and(m_operatorController.rightTrigger())
        // .and(m_elevator.elevatorIsAtTrap())
        // .and(m_pivot.isNearTrapScoringAngle())
        // .onTrue(new ParallelCommandGroup(
        // m_ampTrap.setVelocityCommand(AmpTrapConstants.kTrapSpeed),
        // m_shooter.shootIntoAmpCommand()))
        // .onFalse(new ParallelCommandGroup(
        // m_ampTrap.stopCommand(),
        // m_shooter.stopCommand()));
    }

    private void configureProgrammerBindings() {
    }

    private void configureLights() {
        m_feeder.beambreakIsObstructed()
                .onTrue(m_blinkin.addStateCommand(BlinkinState.NOTE))
                .onFalse(m_blinkin.removeStateCommand(BlinkinState.NOTE));

        m_feeder.beambreakIsObstructed().and(BobotState.inRangeOfSpeakerInterpolation())
                .onTrue(m_blinkin.addStateCommand(BlinkinState.IN_RANGE))
                .onFalse(m_blinkin.removeStateCommand(BlinkinState.IN_RANGE));

        BobotState.pastOppWing().and(m_feeder.beambreakIsObstructed())
                .onTrue(m_blinkin.addStateCommand(BlinkinState.PAST_OPP_WING))
                .onFalse(m_blinkin.removeStateCommand(BlinkinState.PAST_OPP_WING));

    }

    /**
     * Register the commands with PathPlanner
     */
    private void configureNamedCommands() {
        NamedCommands.registerCommand("RunIntakeReal",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                m_feeder.setVelocityBeambreakCommand(
                                        FeederConstants.kIntakeVelocity),
                                m_intake.setPercentOutputCommand(
                                        IntakeConstants.kIntakePercent)),
                        m_intake.stopCommand()));

        NamedCommands.registerCommand("Intake3Seconds",
                new ParallelDeadlineGroup(
                        Commands.waitSeconds(3.0),
                        m_feeder.setVelocityBeambreakCommand(FeederConstants.kIntakeVelocity),
                        m_intake.setPercentOutputCommand(IntakeConstants.kIntakePercent)));

        // Run Interpolation in Parallel
        NamedCommands.registerCommand(
                "Interpolate",
                new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.rampUpSpeedToSpeakerCommand()));

        NamedCommands.registerCommand(
                "InterpolateSetpoint",
                new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_shooter.rampUpSpeedToSpeakerCommand()));

        NamedCommands.registerCommand(
                "AimAtSpeaker",
                Commands.defer(() -> new StrafeAndAimToSpeaker(
                        () -> 0,
                        () -> 0,
                        m_robotDrive),
                        Set.of(m_robotDrive)));

        NamedCommands.registerCommand("InterpolatePivot", m_pivot.controlGoalToSpeakerCommand());

        NamedCommands.registerCommand("InterpolatePivotSetpoint", m_pivot.controlGoalToSpeakerCommand());

        NamedCommands.registerCommand("RampShooter", m_shooter.rampUpSpeedToSpeakerCommand());

        NamedCommands.registerCommand("PivotPID", m_pivot.runTrapezoidProfileCommand());

        NamedCommands.registerCommand(
                "Shoot",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(1),
                                m_feeder.setVelocityCommand(
                                        FeederConstants.kShootVelocity),
                                m_shooter.rampUpSpeedToSpeakerCommand()),
                        m_feeder.stopCommand()));

        NamedCommands.registerCommand(
                "FIRE!",
                new ParallelCommandGroup(
                        m_pivot.controlGoalToSpeakerCommand(),
                        m_intake.setPercentOutputCommand(IntakeConstants.kIntakePercent),
                        m_shooter.rampUpSpeedToSpeakerCommand(),
                        m_feeder.setVelocityCommand(FeederConstants.kShootVelocity)));

        NamedCommands.registerCommand(
                "FireOne",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(1),
                                m_pivot.controlGoalToSpeakerCommand(),
                                m_intake.setPercentOutputCommand(IntakeConstants.kIntakePercent),
                                m_shooter.rampUpSpeedToSpeakerCommand(),
                                m_feeder.setVelocityCommand(FeederConstants.kShootVelocity)),
                        m_feeder.stopCommand()));

        NamedCommands.registerCommand(
                "Fire75",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.75),
                                m_pivot.controlGoalToSpeakerCommand(),
                                m_intake.setPercentOutputCommand(IntakeConstants.kIntakePercent),
                                m_shooter.rampUpSpeedToSpeakerCommand(),
                                m_feeder.setVelocityCommand(FeederConstants.kShootVelocity)),
                        m_feeder.stopCommand()));

        NamedCommands.registerCommand(
                "NewFireOne",
                Commands.sequence(
                        Commands.waitSeconds(0.75),
                        m_feeder.setVelocityCommand(FeederConstants.kShootVelocity),
                        Commands.waitSeconds(0.25),
                        m_feeder.stopCommand()));

        NamedCommands.registerCommand(
                "NewFireHs",
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        m_feeder.setVelocityCommand(FeederConstants.kShootVelocity),
                        Commands.waitSeconds(0.25),
                        m_feeder.stopCommand()));

        NamedCommands.registerCommand(
                "TargetNote",
                new InstantCommand(() -> BobotState.updateAimingMode(AimingMode.OBJECT_DETECTION)));

        NamedCommands.registerCommand(
                "TargetSpeaker",
                new InstantCommand(() -> BobotState.updateAimingMode(AimingMode.SPEAKER)));

        NamedCommands.registerCommand(
                "TargetRotation",
                new InstantCommand(() -> BobotState.updateAimingMode(AimingMode.NONE)));

        NamedCommands.registerCommand(
                "SubwayShot",
                Commands.sequence(
                        m_shooter.setVelocityCommand(
                                ShooterConstants.kLeftShooterSpeed / 1.5,
                                ShooterConstants.kRightShooterSpeed / 1.5),
                        m_pivot.setGoalCommand(PivotLocation.kSubwooferScoringPosition.angle),
                        // Commands.waitUntil(m_pivot.isNearGoal()),
                        Commands.waitSeconds(0.75),
                        m_feeder.setVelocityCommand(FeederConstants.kShootVelocity),
                        Commands.waitSeconds(0.4)));

        // New Auto Structure Commands:
        // Using event markers in pp/choreo, we can move any commands we want run
        // during paths there and only using explicit calls during auto creation for
        // commands run in between paths

        // Intake on/off
        NamedCommands.registerCommand(
                "IntakeOn",
                Commands.parallel(
                        m_feeder.setVelocityBeambreakCommand(FeederConstants.kIntakeVelocity),
                        m_intake.setPercentOutputCommand(IntakeConstants.kIntakePercent)));
        NamedCommands.registerCommand(
                "IntakeOff",
                Commands.parallel(
                        m_feeder.stopCommand(),
                        m_intake.stopCommand()));

        // ramp up / interpolate on/off
        NamedCommands.registerCommand(
                "ShotPrepare",
                Commands.parallel(
                        m_shooter.rampUpSpeedToSpeakerCommand(),
                        m_pivot.controlGoalToSpeakerCommand()));

        NamedCommands.registerCommand(
                "AtEase",
                Commands.parallel(
                        m_shooter.stopCommand(),
                        m_pivot.setEverythingCurrentCommand()));

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

        Command otherAmpCommand = Commands.defer(() -> new PositionWithAmp(
                () -> -m_driverController.getLeftY(),
                m_robotDrive,
                OffsetTags.OTHER_AMP),
                Set.of(m_robotDrive));

        // Each command that we plan to use for 'Lane Assist' should be deferred
        // with their respective subsystems. Once they're deferred, we can then
        // proxy the deferred command to run while the button is held.
        m_laneAssistChooser = new LoggedDashboardChooser<>("LaneAssist", new SendableChooser<>());

        m_laneAssistCommands.put("Amp",
                new LaneAssist(m_automation.ampPath(), m_automation.ampAssist()));
        m_laneAssistCommands.put("Other Amp",
                new LaneAssist(OffsetTags.OTHER_AMP.getDeferredCommand(), otherAmpCommand));
        m_laneAssistCommands.put("Human Player",
                new LaneAssist(PathPlannerPoses.HUMAN_PLAYER.getDeferredCommand(),
                        new InstantCommand()));
        m_laneAssistCommands.put("Aim at Speaker",
                new LaneAssist(Commands.none(), m_automation.aimAtSpeakerAssist()));
        m_laneAssistCommands.put("Stage Center",
                new LaneAssist(m_automation.stageCenterPath(), m_automation.stageCenterAssist()));
        m_laneAssistCommands.put("Stage Human",
                new LaneAssist(m_automation.stageHumanPath(), m_automation.stageHumanPath()));
        m_laneAssistCommands.put("Stage Amp",
                new LaneAssist(m_automation.stageAmpPath(), m_automation.stageAmpAssist()));
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
