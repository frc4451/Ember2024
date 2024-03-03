package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimberVisualizer {
    private final Mechanism2d mechanism;
    private final MechanismLigament2d climber;
    private final String key;

    private final double climberLength = Units.inchesToMeters(20);
    private final Translation2d climberOrigin = new Translation2d(0, 0);

    public ClimberVisualizer(String key, Color color) {
        this.key = key;
        mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
        MechanismRoot2d root = mechanism.getRoot("climber", 1.0, 0.4);
        climber = new MechanismLigament2d("climber", climberLength, 90.0, 6, new Color8Bit(color));

        climber
                .append(
                        new MechanismLigament2d(
                                "climber_stage",
                                Units.inchesToMeters(6),
                                90,
                                6,
                                new Color8Bit(color)))
                .append(
                        new MechanismLigament2d(
                                "climber_hook",
                                Units.inchesToMeters(6),
                                90,
                                6,
                                new Color8Bit(color)));
        root.append(climber);
    }

    /** Update climber visualizer with current climber height */
    public void update(double positionInches) {
        climber.setLength(Units.inchesToMeters(positionInches));
        Logger.recordOutput("Climber/Mechanisms/" + key + "/Mechanism2d", mechanism);

        // Log 3d poses
        Pose3d elevator = new Pose3d(
                climberOrigin.getX(),
                climberOrigin.getY() + this.climber.getLength(),
                0.0,
                new Rotation3d());
        Logger.recordOutput("Climber/Mechanisms/" + key + "/Pose3d", elevator);
    }
}
