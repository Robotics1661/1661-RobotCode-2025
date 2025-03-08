package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.AutonomousDebugger;

public class IntakeCoralAutoCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final Timer m_sim_end_timer;

    public IntakeCoralAutoCommand(
        IntakeSubsystem intake
    ) {
        this.m_intake = intake;

        if (Utils.isSimulation()) {
            m_sim_end_timer = new Timer();
        } else {
            m_sim_end_timer = null;
        }

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        AutonomousDebugger.autoDbg("Starting coral intake");
        if (Utils.isSimulation()) {
            m_sim_end_timer.reset();
            m_sim_end_timer.start();
        }
    }

    @Override
    public void execute() {
        m_intake.runIntake(0.3);
    }

    @Override
    public boolean isFinished() {
        if (Utils.isSimulation()) {
            return m_sim_end_timer.hasElapsed(1);
        }
        return m_intake.coralReachedEnd();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.runIntake(0.0);
        AutonomousDebugger.autoDbg("Done with coral intake");
    }
}
