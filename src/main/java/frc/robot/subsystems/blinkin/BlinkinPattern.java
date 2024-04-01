package frc.robot.subsystems.blinkin;

public enum BlinkinPattern {
    SOLID(Double.MAX_VALUE), // this will take 5.7e300 years to blink
    BLINK(0.1);

    public final boolean blinks;
    public final double blinkIntervalSeconds;

    private BlinkinPattern(double blinkIntervalSeconds) {
        this.blinks = true;
        this.blinkIntervalSeconds = blinkIntervalSeconds;
    }

    private BlinkinPattern() {
        this.blinks = false;
        this.blinkIntervalSeconds = Double.NaN;
    }

}
