package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberManualCommand extends Command {
    private final ClimberSubsystem m_climber;
    private final BooleanSupplier m_extend_climb;
    private final BooleanSupplier m_retract_climb;

    public ClimberManualCommand(
        ClimberSubsystem climber,
        BooleanSupplier extend_climb,
        BooleanSupplier retract_climb
    ) {
        this.m_climber = climber;
        this.m_extend_climb = extend_climb;
        this.m_retract_climb = retract_climb;

        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        if (m_extend_climb.getAsBoolean()) {
            m_climber.extend();
        } else if (m_retract_climb.getAsBoolean()) {
            m_climber.retract();
        } else {
            m_climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
