package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.AdvantageKitConstants.Mode;

public final class VisionConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static record VisionSource(String name, Transform3d robotToCamera) {
    }

    public static final Optional<VisionSystemSim> VISION_SYSTEM_SIM = AdvantageKitConstants.getMode() == Mode.SIM
            ? Optional.of(new VisionSystemSim("VisionSim"))
            : Optional.empty();

    // Establish all Cameras, their names, and where they are on the robot.
    //
    // NOTE this should be updated with real values from measurements on the robot
    public static final List<VisionSource> APRIL_TAG_SOURCES = List.of(
            new VisionSource(
                    "Arducam_OV9281_USB_Camera",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(-2.75), // right+
                                    Units.inchesToMeters(-12), // foward+
                                    Units.inchesToMeters(8.5)), // up+
                            new Rotation3d(
                                    0, Units.degreesToRadians(-20),
                                    Units.degreesToRadians(180 + 30)))),
            new VisionSource(
                    "RG_Camera_2",
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(2.75), // right+
                                    Units.inchesToMeters(-12), // foward+
                                    Units.inchesToMeters(8.5)), // up+
                            new Rotation3d(
                                    0, Units.degreesToRadians(-20),
                                    Units.degreesToRadians(180 - 30)))));

    public static final List<VisionSource> OBJECT_DETECTION_SOURCES = List.of(
            // This isn't going to be installed on the robot when we start, but this is the
            // definition when we need it
            new VisionSource("ObjectDetectionCamera", new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(0),
                            Units.inchesToMeters(-12),
                            Units.inchesToMeters(8.5)),
                    new Rotation3d(0, Units.degreesToRadians(-5), 0))));

    // Review the Field layout for positions
    // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    public static final Set<Integer> RED_HUMAN_PLAYER_TAGS = Set.of(1, 2);
    public static final Set<Integer> RED_SPEAKER_TAGS = Set.of(3, 4);
    public static final Set<Integer> RED_AMP_TAG = Set.of(5);

    public static final Set<Integer> BLUE_AMP_TAG = Set.of(6);
    public static final Set<Integer> BLUE_SPEAKER_TAGS = Set.of(7, 8);
    public static final Set<Integer> BLUE_HUMAN_PLAYER_TAGS = Set.of(9, 10);

    public static final Set<Integer> RED_STAGE_TAGS = Set.of(11, 12, 13);
    public static final Set<Integer> BLUE_STAGE_TAGS = Set.of(14, 15, 16);

    public static final Set<Integer> RED_TAG_FIDS = Stream
            .of(RED_HUMAN_PLAYER_TAGS, RED_SPEAKER_TAGS, RED_AMP_TAG, RED_STAGE_TAGS)
            .flatMap(Set::stream)
            .collect(Collectors.toSet());

    public static final Set<Integer> BLUE_TAG_FIDS = Stream
            .of(BLUE_HUMAN_PLAYER_TAGS, BLUE_SPEAKER_TAGS, BLUE_AMP_TAG, BLUE_STAGE_TAGS)
            .flatMap(Set::stream)
            .collect(Collectors.toSet());

    public static final Set<Integer> ALL_TAGS = Stream.concat(RED_TAG_FIDS.stream(), BLUE_TAG_FIDS.stream())
            .collect(Collectors.toSet());

    // Shorthand for generating all possible combinations of April Tags
    public static final List<Set<Integer>> POSSIBLE_FRAME_FID_COMBOS = List.of(RED_TAG_FIDS, BLUE_TAG_FIDS);

    // This is a guess, feel free to update.
    public static final int MAX_FRAME_FIDS = 8;
    public static final double POSE_AMBIGUITY_CUTOFF = .05;

    /***********************************************************************
     * Represents parameters for computing unit deviation
     * based on average distance.
     *
     * The record encapsulates three parameters:
     * - distanceMultiplier: Multiplier applied to the average distance.
     * - eulerMultiplier: Multiplier applied to the exponential term.
     * - minimum: Minimum value for the computed unit deviation.
     *
     * ComputeUnitDeviation calculates unit deviation using the
     * provided average distance according to the formula:
     *
     * max(minimum, eulerMultiplier * exp(averageDistance * distanceMultiplier)).
     */
    public static record UnitDeviationParams(
            double distanceMultiplier,
            double eulerMultiplier,
            double minimum) {
        /**
         * Computes unit deviation based on the average distance param.
         *
         * @param averageDistance The average distance used in the computation.
         * @return The computed unit deviation.
         */
        private double ComputeUnitDeviation(double averageDistance) {
            return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
        }
    }

    /***********************************************************************
     * Represents deviation parameters for tags in X, Y, and Theta.
     *
     * This record encapsulates three sets of deviation parameters:
     * - xParams: Parameters for X dimension.
     * - yParams: Parameters for Y dimension.
     * - thetaParams: Parameters for Theta.
     *
     * The ComputeDeviation method calculates the deviation matrix based
     * on the provided average distance. The resulting matrix represents
     * deviations for X, Y, and Theta dimensions.
     *
     * The constructor TagCountDeviation(xyParams, thetaParams)
     * initializes the deviation parameters for X and Y dimensions
     * using the same set of parameters.
     */
    public static record TagCountDeviation(
            UnitDeviationParams xParams,
            UnitDeviationParams yParams,
            UnitDeviationParams thetaParams) {
        /**
         * Computes the deviation matrix based on the average distance parameter.
         *
         * @param averageDistance The average distance used in the computation.
         * @return The deviation matrix for X, Y, and Theta dimensions.
         */
        public Matrix<N3, N1> computeDeviation(double averageDistance) {
            return MatBuilder.fill(
                    Nat.N3(),
                    Nat.N1(),
                    xParams.ComputeUnitDeviation(averageDistance),
                    yParams.ComputeUnitDeviation(averageDistance),
                    thetaParams.ComputeUnitDeviation(averageDistance));
        }

        /**
         * Constructor that initializes deviation parameters for X and Y dimensions
         * using the same set
         * of parameters.
         *
         * @param xyParams    Parameters for X and Y dimensions.
         * @param thetaParams Parameters for Theta dimension.
         */
        public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
            this(xyParams, xyParams, thetaParams);
        }
    }

    /**
     * The TAG_COUNT_DEVIATION_PARAMS list contains instances of TagCountDeviation,
     * each
     * representing deviation parameters for tags in X, Y, and Theta dimensions
     * based on the number
     * of tags detected. The list is organized by the count of tags: - For 1 tag:
     * Parameters for X,
     * Y, and Theta dimensions are specified. - For 2 tags: Parameters for X and Y
     * dimensions are
     * specified, using the same set of parameters. - For 3 or more tags: Parameters
     * for X and Y
     * dimensions are specified, using the same set of parameters.
     *
     * <p>
     * Each TagCountDeviation instance contains three sets of UnitDeviationParams: -
     * xParams:
     * Parameters for X dimension. - yParams: Parameters for Y dimension. -
     * thetaParams: Parameters
     * for Theta dimension.
     *
     * <p>
     * The deviation matrix can be computed using the ComputeDeviation method for a
     * given average
     * distance. The resulting matrix represents deviations for X, Y, and Theta
     * dimensions.
     *
     * <p>
     * Example Usage: List<TagCountDeviation> deviations =
     * TAG_COUNT_DEVIATION_PARAMS;
     * TagCountDeviation paramsFor1Tag = deviations.get(0); Matrix<N3, N1>
     * deviationMatrix =
     * paramsFor1Tag.ComputeDeviation(averageDistance);
     */
    public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS = List.of(
            // 1 tag
            new TagCountDeviation(
                    new UnitDeviationParams(.25, .4, .9),
                    new UnitDeviationParams(.35, .5, 1.2),
                    new UnitDeviationParams(.5, .7, 1.5)),

            // 2 tags
            new TagCountDeviation(
                    new UnitDeviationParams(.35, .1, .4),
                    new UnitDeviationParams(.5, .7, 1.5)),

            // 3+ tags
            new TagCountDeviation(
                    new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)));
}
