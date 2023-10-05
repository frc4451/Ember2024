package frc.robot.subsystems;

public enum RollerMode {
    SUCK(-0.1, -0.1, true),
    SHOOT(0.1, 0.1, false);

    public double topPercent;
    public double bottomPercent;
    public boolean isLimited;

    RollerMode(double topPercent, double bottomPercent, boolean isLimited) {
        this.topPercent = topPercent;
        this.bottomPercent = bottomPercent;
        this.isLimited = isLimited;
    }
}
