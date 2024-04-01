package frc.robot.subsystems.blinkin;

public enum BlinkinState {
    DEFAULT(BlinkinColors.UNKNOWN, BlinkinPattern.SOLID),
    NOTE(BlinkinColors.SOLID_ORANGE, BlinkinPattern.BLINK);

    public final BlinkinColors color;
    public final BlinkinPattern pattern;

    private BlinkinState(BlinkinColors color, BlinkinPattern pattern) {
        this.color = color;
        this.pattern = pattern;
    }
}
