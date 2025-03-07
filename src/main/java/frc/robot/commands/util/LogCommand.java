package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LogCommand extends InstantCommand {
    public LogCommand(String msg) {
        super(() -> System.out.println(msg));
    }
}
