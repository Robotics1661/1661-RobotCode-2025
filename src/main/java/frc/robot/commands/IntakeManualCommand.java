package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier m_intake_supplier;
    private final BooleanSupplier m_drop_supplier;

    public IntakeManualCommand(
        IntakeSubsystem intake,
        BooleanSupplier intake_supplier,
        BooleanSupplier drop_supplier
    ) {
        this.m_intake = intake;
        this.m_intake_supplier = intake_supplier;
        this.m_drop_supplier = drop_supplier;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        if (m_intake_supplier.getAsBoolean()) {
            m_intake.runIntake(0.3);
        } else if (m_drop_supplier.getAsBoolean()) {
            m_intake.runIntake(-0.25);
        } else {
            m_intake.runIntake(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableStop();
    }
}
