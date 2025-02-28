package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristManualCommand extends Command {
    private final WristSubsystem m_wrist;
    private final DoubleSupplier m_directionInput;

    public WristManualCommand(WristSubsystem wrist, DoubleSupplier directionInput) {
        this.m_wrist = wrist;
        this.m_directionInput = directionInput;

        addRequirements(m_wrist);
    }

    @Override
    public void execute() {
        super.execute();

        m_wrist.smartMoveRate(m_directionInput.getAsDouble() * -0.25);
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.disableStop();
    }
}
