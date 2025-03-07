package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

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
        if (Utils.isSimulation()) {
            System.out.println("Running intake at rate "+m_rate);
        }
    }

    @Override
    public void execute() {
        m_intake.runIntake(m_rate);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.disableStop();
        if (Utils.isSimulation()) {
            System.out.println("Done running intake");
        }
    }
}
