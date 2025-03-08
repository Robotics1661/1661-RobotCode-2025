package frc.robot.subsystems;

import static frc.robot.util.MathUtil.clamp;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class IntakeSubsystem extends CustomSubsystemBase {
    private final TalonFX m_intake = new TalonFX(Constants.MANIPULATOR_INTAKE);

    // Period is 0.02 seconds - this is the standard FRC main loop period
    private final LinearFilter m_intake_current_filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private double m_intake_rate = 0.0;
    /** Whether the coral/algae has reached the back of the intake. Detected by current spikes */
    private boolean m_intake_reached = false;

    public IntakeSubsystem() {
        setSubsystem("intake");

        ntPutData(builder -> {
            builder.addDoubleProperty("current (Amps)", () -> m_intake.getTorqueCurrent().getValueAsDouble(), null);
            builder.addDoubleProperty("filt current (Amps)", () -> m_intake_current_filter.lastValue(), null);
            builder.addBooleanProperty("spiking", () -> isIntakeCurrentSpiking(), null);
            builder.addBooleanProperty("back reached", () -> m_intake_reached, null);
        });

        var intakeConf = m_intake.getConfigurator();
        intakeConf.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // positive means intake
            .withNeutralMode(NeutralModeValue.Brake)
        );
    }

    @Override
    public void periodic() {
        m_intake_current_filter.calculate(m_intake.getTorqueCurrent().getValueAsDouble());

        if (!m_intake_reached && isIntakeCurrentSpiking()) {
            m_intake_reached = true;
            m_intake.setControl(STATIC_BRAKE);
        }
    }

    public boolean coralReachedEnd() {
        return m_intake_reached;
    }

    // motion methods

    public void runIntake(double rate) {
        runIntake(rate, false);
    }

    public void runIntake(double rate, boolean skipCurrentLimit) {
        rate = clamp(rate, -1, 1);

        if (Math.abs(rate - m_intake_rate) > 0.01) {
            m_intake_reached = false;
        }

        m_intake_rate = rate;
        if (skipCurrentLimit) {
            m_intake_reached = false;
            m_intake_rate = 0.0; // prevent current spike code from running
        }

        if (!m_intake_reached) {
            if (Math.abs(rate) > 0.01) {
                m_intake.setControl(getPooledVoltageOut(m_intake, Units.Volt.of(rate * 12.0)));
            } else {
                m_intake.setControl(STATIC_BRAKE);
            }
        }
    }

    public void disableStop() {
        m_intake.setControl(COAST_OUT);
    }

    // Limits

    protected boolean isIntakeCurrentSpiking() {
        if (m_intake_rate <= 0.0) { // irrelevant when outputting
            return false;
        }
        return m_intake_current_filter.lastValue() > 60.0;
    }
}
