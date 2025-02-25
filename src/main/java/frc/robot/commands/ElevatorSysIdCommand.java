package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSysIdCommand {

    private static class LogCommand extends InstantCommand {
        public LogCommand(String msg) {
            super(() -> System.out.println(msg));
        }
    }

    public static Command create(ElevatorSubsystem elevator, BooleanSupplier safety) {
        return new SequentialCommandGroup(
            /*new ElevatorCallibrationCommand(elevator),
            new LogCommand("callibration done"),*/
            elevator.createSysIdRoutine(3).quasistatic(Direction.kForward),
            new LogCommand("quasistatic forward done"),
            elevator.createSysIdRoutine(3).quasistatic(Direction.kReverse),
            new LogCommand("quasistatic reverse done"),
            elevator.createSysIdRoutine(3.0).dynamic(Direction.kForward),
            new LogCommand("dynamic forward done"),
            elevator.createSysIdRoutine(3.0).dynamic(Direction.kReverse),
            new LogCommand("dynamic reverse done")
        ).onlyWhile(safety);
    }
}
