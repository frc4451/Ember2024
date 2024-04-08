package frc.robot.bobot_state;

import frc.robot.bobot_state.ShootingInterpolator.InterpolatedCalculation;

// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa, sometimes i want pythonic typed lists
public class InterpolationTriplet {
    public final String name;
    public final TargetInterpolator interpolator;
    public InterpolatedCalculation calculation;

    public InterpolationTriplet(String name, TargetInterpolator interpolator) {
        this.name = name;
        this.interpolator = interpolator;
    }
}
