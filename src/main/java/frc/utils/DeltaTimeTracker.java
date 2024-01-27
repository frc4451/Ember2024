package frc.utils;

public class DeltaTimeTracker {
    private double lastTimestamp = 0.0;
    private double deltaTime = 0.0;

    public void update(double timestamp) {
        deltaTime = lastTimestamp - timestamp;
        lastTimestamp = timestamp;
    }

    public double get() {
        return deltaTime;
    }
}
