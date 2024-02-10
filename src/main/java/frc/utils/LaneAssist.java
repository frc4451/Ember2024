package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;

public record LaneAssist(Command pathfindCommand, Command aimingCommand) {
};
