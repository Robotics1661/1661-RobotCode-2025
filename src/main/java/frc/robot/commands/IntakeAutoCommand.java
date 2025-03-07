package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.AutonomousDebugger;

public class IntakeAutoCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_rate;

    public IntakeAutoCommand(
        IntakeSubsystem intake,
        double rate
    ) {
        this.m_intake = intake;
        this.m_rate = rate;

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        AutonomousDebugger.autoDbg("Running intake at rate "+m_rate);
    }

    @Override
    public void execute() {
        m_intake.runIntake(m_rate);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableStop();
        AutonomousDebugger.autoDbg("Done running intake");
    }
}
