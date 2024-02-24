package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotVisualizer {
    private final Mechanism2d mechanism;
    private final MechanismLigament2d pivot;
    private final String key;

    // This is guesswork, Allred can fill this in more accurately.
    private final double pivotLength = Units.inchesToMeters(26);
    private final Translation2d pivotOrigin = new Translation2d(0, 0);

    public PivotVisualizer(String key, Color color) {
        this.key = key;
        mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
        MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
        pivot = new MechanismLigament2d("pivot", pivotLength, 20.0, 6, new Color8Bit(color));
        root.append(pivot);
    }

    /** Update arm visualizer with current arm angle */
    public void update(double angleRads) {
        pivot.setAngle(Rotation2d.fromRadians(angleRads));
        Logger.recordOutput("Pivot/Mechanisms/" + key + "/Mechanism2d", mechanism);

        // Log 3d poses
        Pose3d pivot = new Pose3d(pivotOrigin.getX(), 0.0, pivotOrigin.getY(), new Rotation3d(0.0, -angleRads, 0.0));
        Logger.recordOutput("Pivot/Mechanisms/" + key + "/Pose3d", pivot);
    }
}
