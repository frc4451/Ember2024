package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandCustomController extends CommandXboxController {
    private final double joystickDeadband = 0.2;
    private final double triggerPressedThreshold = 0.1;

    /**
     * Construct an instance of a command controller.
     *
     * @param port The port on Driver Station where the controller is connected.
     */
    public CommandCustomController(int port) {
        super(port);
    }

    /**
     *
     * {@inheritDoc} (Processed with function below).
     *
     * @see #processJoystickValue
     */
    @Override
    public double getLeftX() {
        return this.processJoystickValue(super.getLeftX());
    }

    /**
     *
     * {@inheritDoc} (Processed with function below).
     *
     * @see #processJoystickValue
     */
    @Override
    public double getLeftY() {
        return this.processJoystickValue(super.getLeftY());
    }

    /**
     *
     * {@inheritDoc} (Processed with function below).
     *
     * @see #processJoystickValue
     */
    @Override
    public double getRightX() {
        return this.processJoystickValue(super.getRightX());
    }

    /**
     *
     * {@inheritDoc} (Processed with function below).
     *
     * @see #processJoystickValue
     */
    @Override
    public double getRightY() {
        return this.processJoystickValue(super.getRightY());
    }

    /**
     * @return Trigger instance that is true when the Left Stick's X is not 0.
     */
    public Trigger leftX() {
        return new Trigger(() -> this.getLeftX() != 0);
    }

    /**
     * @return Trigger instance that is true when the Left Stick's Y is not 0.
     */
    public Trigger leftY() {
        return new Trigger(() -> this.getLeftY() != 0);
    }

    /**
     * @return Trigger instance that is true when the Right Stick's X is not 0.
     */
    public Trigger rightX() {
        return new Trigger(() -> this.getRightX() != 0);
    }

    /**
     * @return Trigger instance that is true when the Right Stick's Y is not 0.
     */
    public Trigger rightY() {
        return new Trigger(() -> this.getRightY() != 0);
    }

    /**
     * NOTE: Trigger Treshold has been overriden by {@link CommandCustomController},
     * check see block below.<br>
     * ORIGINAL DOCS: {@inheritDoc}
     *
     * @see #triggerPressedThreshold
     */
    @Override
    public Trigger leftTrigger() {
        return this.leftTrigger(this.triggerPressedThreshold);
    }

    /**
     * NOTE: Trigger Treshold has been overriden by {@link CommandCustomController},
     * check see block below.<br>
     * ORIGINAL DOCS: {@inheritDoc}
     *
     * @see #triggerPressedThreshold
     */
    @Override
    public Trigger rightTrigger() {
        return this.rightTrigger(this.triggerPressedThreshold);
    }

    /**
     * Processes joystick value like so: { (deadband(v)^2) * signum(v) }.<br>
     * The squaring is done to improve precision.
     * The signum multiplication is to get the negativeness back (if it was there).
     *
     * @param value Value to process
     * @return Processed value
     */
    private double processJoystickValue(double value) {
        return Math.pow(applyJoystickDeadband(value), 2) * Math.signum(value);
    }

    /**
     * Applies a predefined deadband to a value. Meant for joysticks.
     *
     * @param stickValue Value of a joystick, usually [-1.0, 1.0]
     * @return Joystick's value with a deadband applied
     */
    private double applyJoystickDeadband(double stickValue) {
        return MathUtil.applyDeadband(stickValue, this.joystickDeadband);
    }
}
