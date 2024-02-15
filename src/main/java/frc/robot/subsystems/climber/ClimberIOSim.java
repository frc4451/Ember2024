package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
    public void updateInputs(ClimberIOInputs inputs) {
    }

    public void setVelocity(double velocityRotPerSecond) {
    }

    public void setVoltage(double voltage) {
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void setPosition(double position) {

    }
}
