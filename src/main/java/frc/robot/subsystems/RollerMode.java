package frc.robot.subsystems;

public enum RollerMode {
    STOP(0.0, 0.0, false),
    SUCK(-0.5, -0.5, true),
    SHOOTLOW(0.3, 0.3, false),
    SHOOTMID(0.7, 0.7, false),
    SHOOTHIGH(1, 1, false);

    public double topPercent;
    public double bottomPercent;
    public boolean isLimited;

    RollerMode(double topPercent, double bottomPercent, boolean isLimited) {
        this.topPercent = topPercent;
        this.bottomPercent = bottomPercent;
        this.isLimited = isLimited;
    }
}
