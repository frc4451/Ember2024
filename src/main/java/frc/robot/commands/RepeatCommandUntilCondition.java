package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/**
 * Modification of {@link RepeatCommand} that allows us to create a
 * custom "end condition" to override {@link Command#isFinished}
 */
public class RepeatCommandUntilCondition extends RepeatCommand {
    private final BooleanSupplier condition;

    public RepeatCommandUntilCondition(
            Command command,
            BooleanSupplier condition) {
        super(command);
        this.condition = condition;
    }

    public RepeatCommandUntilCondition(
            Supplier<Command> commandSupplier,
            BooleanSupplier condition) {
        super(commandSupplier.get());
        this.condition = condition;
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}
