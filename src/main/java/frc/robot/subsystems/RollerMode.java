package frc.robot.subsystems;

public enum RollerMode {
    STOP(0.0, 0.0, false),
    SUCK(-0.3, -0.3, true),
    SHOOT(0.3, 0.3, false);

    public double topPercent;
    public double bottomPercent;
    public boolean isLimited;

    RollerMode(double topPercent, double bottomPercent, boolean isLimited) {
        this.topPercent = topPercent;
        this.bottomPercent = bottomPercent;
        this.isLimited = isLimited;
    }
}
