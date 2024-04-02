package frc.robot.subsystems.blinkin;

public enum BlinkinState {
    // Priority is determined by definition order,
    // e.g. NOTE has precedence over DEFAULT
    READYSHOOT(BlinkinColors.SOLID_HOT_PINK, BlinkinPattern.SOLID),
    NOTE(BlinkinColors.SOLID_ORANGE, BlinkinPattern.BLINK),
    DEFAULT(BlinkinColors.SOLID_BLUE, BlinkinPattern.SOLID);

    public final BlinkinColors color;
    public final BlinkinPattern pattern;

    private BlinkinState(BlinkinColors color, BlinkinPattern pattern) {
        this.color = color;
        this.pattern = pattern;
    }
}
