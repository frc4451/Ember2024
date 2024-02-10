package frc.robot.subsystems.vision.apriltag;

import frc.robot.VisionConstants;
import frc.utils.GarageUtils;

public enum StageTags {
    HUMAN(VisionConstants.RED_STAGE_HUMAN, VisionConstants.BLUE_STAGE_HUMAN),
    AMP(VisionConstants.RED_STAGE_AMP, VisionConstants.BLUE_STAGE_AMP),
    CENTER(VisionConstants.RED_STAGE_CENTER, VisionConstants.BLUE_STAGE_CENTER);

    private final int red;
    private final int blue;

    private StageTags(int red, int blue) {
        this.red = red;
        this.blue = blue;
    }

    public int get() {
        return GarageUtils.isRedAlliance() ? red : blue;
    }

    public int getBlue() {
        return blue;
    }

    public int getRed() {
        return red;
    }
}
