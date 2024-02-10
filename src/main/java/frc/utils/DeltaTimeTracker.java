package frc.utils;

public class DeltaTimeTracker {
    private double lastTimestamp = 0.0;
    private double deltaTime = 0.0;

    public double update(double timestamp) {
        deltaTime = lastTimestamp - timestamp;
        lastTimestamp = timestamp;

        return get();
    }

    public double get() {
        return deltaTime;
    }
}
