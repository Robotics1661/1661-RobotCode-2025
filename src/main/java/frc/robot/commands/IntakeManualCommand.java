package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeManualCommand extends Command {
    private final ManipulatorSubsystem m_manipulator;
    private final BooleanSupplier m_intake;
    private final BooleanSupplier m_drop;

    public IntakeManualCommand(ManipulatorSubsystem manipulator, BooleanSupplier intake, BooleanSupplier drop) {
        this.m_manipulator = manipulator;
        this.m_intake = intake;
        this.m_drop = drop;

        addRequirements(m_manipulator);
    }

    @Override
    public void execute() {
        if (m_intake.getAsBoolean()) {
            m_manipulator.runIntake(0.1);
        } else if (m_drop.getAsBoolean()) {
            m_manipulator.runIntake(-0.1);
        } else {
            m_manipulator.runIntake(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.runIntake(0);
    }
}
