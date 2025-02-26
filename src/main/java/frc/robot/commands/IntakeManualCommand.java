package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.WristSetPoint;

public class IntakeManualCommand extends Command {
    private final ManipulatorSubsystem m_manipulator;
    private final BooleanSupplier m_intake;
    private final BooleanSupplier m_drop;
    private final BooleanSupplier m_wristTest;
    private final DoubleSupplier m_wristControl;

    public IntakeManualCommand(
        ManipulatorSubsystem manipulator,
        BooleanSupplier intake,
        BooleanSupplier drop,
        BooleanSupplier wristTest,
        DoubleSupplier wristControl
    ) {
        this.m_manipulator = manipulator;
        this.m_intake = intake;
        this.m_drop = drop;
        this.m_wristTest = wristTest;
        this.m_wristControl = wristControl;

        addRequirements(m_manipulator);
    }

    @Override
    public void execute() {
        if (m_intake.getAsBoolean()) {
            m_manipulator.runIntake(0.3);
        } else if (m_drop.getAsBoolean()) {
            m_manipulator.runIntake(-0.15);
        } else {
            m_manipulator.runIntake(0.0);
        }

        double wristControl = m_wristControl.getAsDouble();
        if (m_wristTest.getAsBoolean()) {
            m_manipulator.moveWristTo(WristSetPoint.TEST_LOW);
        } else if (Math.abs(wristControl) > 0.1) {
            m_manipulator.moveWrist(wristControl * 0.1);
        } else {
            m_manipulator.stopWrist();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.runIntake(0);
    }
}
