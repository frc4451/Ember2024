package frc.utils;

import java.util.Map;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Creates a logged Finite State Machine controlled by Command button inputs
 * via SmartDashboard. All inputs are logged to AdvantageKit, but controlled
 * by SmartDashboard via their `putData(String key, Command command)`
 * method.
 */
public class LoggedDashboardButtonFSM implements LoggedDashboardInput {
    private final String key;
    private final Map<String, Command> stateCommands;

    private String currentState;

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, currentState);
        }

        public void fromLog(LogTable table) {
            currentState = table.get(key, currentState);
        }
    };

    public LoggedDashboardButtonFSM(String key, Map<String, Command> stateCommands) {
        this.key = key;
        this.stateCommands = stateCommands;

        // Assumes that the user is using a `LinkedHashMap`
        this.currentState = this.stateCommands.keySet().iterator().next();

        this.stateCommands.forEach((String name, Command command) -> {
            SmartDashboard.putData(key + "/" + name, new InstantCommand(() -> {
                this.currentState = name;
            }));
        });

        periodic();

        Logger.registerDashboardInput(this);
    }

    @Override
    public void periodic() {
        Logger.processInputs(prefix, inputs);
    }

    /**
     * Get the Command currently selected by the state machine.
     *
     * @return Currently Selected Command
     */
    public Command getCurrentCommand() {
        return this.stateCommands.get(currentState);
    }
}
