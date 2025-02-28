package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier m_intake_coral;
    private final BooleanSupplier m_intake_algae;
    private final BooleanSupplier m_drop;
    private final BooleanSupplier m_drop_slow;

    public IntakeManualCommand(
        IntakeSubsystem intake,
        BooleanSupplier intake_coral,
        BooleanSupplier intake_algae,
        BooleanSupplier drop,
        BooleanSupplier drop_slow
    ) {
        this.m_intake = intake;
        this.m_intake_coral = intake_coral;
        this.m_intake_algae = intake_algae;
        this.m_drop = drop;
        this.m_drop_slow = drop_slow;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        if (m_intake_coral.getAsBoolean()) {
            m_intake.runIntake(0.3);
        } else if (m_intake_algae.getAsBoolean()) {
            m_intake.runIntake(0.35, true);
        } else if (m_drop.getAsBoolean()) {
            m_intake.runIntake(-0.25);
        } else if (m_drop_slow.getAsBoolean()) {
            m_intake.runIntake(-0.125);
        } else {
            m_intake.runIntake(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableStop();
    }
}
