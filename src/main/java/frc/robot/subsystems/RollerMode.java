package frc.robot.subsystems;

public enum RollerMode {
    STOP(0.0, 0.0, false),
    SUCK(-0.5, -0.5, true),
    SHOOT_LOW(0.3, 0.3, false),
    SHOOT_MID(0.7, 0.7, false),
    SHOOT_HIGH(1.0, 1.0, false);

    public double topPercent;
    public double bottomPercent;
    public boolean isLimited;

    RollerMode(double topPercent, double bottomPercent, boolean isLimited) {
        this.topPercent = topPercent;
        this.bottomPercent = bottomPercent;
        this.isLimited = isLimited;
    }
}
