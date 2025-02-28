package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoyXboxWrapper {
    private final Optional<XboxController> m_xbox;
    private final Optional<Joystick> m_joystick; //flightstick

    private boolean m_soft_disabled;
    private final boolean m_auto_swap_mode = true; // If true, soft enable if twist is on +

    public JoyXboxWrapper(int xboxPort, int joystickPort) {
        if (Utils.isSimulation()) {
            m_xbox = Optional.empty();
            m_joystick = Optional.empty();
        } else {
            m_xbox = Optional.of(new XboxController(xboxPort));
            m_joystick = Optional.of(new Joystick(joystickPort));
        }
        m_soft_disabled = true;
    }

    // region movement

    public double getLateralX() {
        if (Constants.DISABLE_SWERVE) return 0;
        return isSoftDisabled() ? 0 : m_joystick.map(Joystick::getX).orElse(0.0);
    }

    public double getLateralY() {
        if (Constants.DISABLE_SWERVE) return 0;
        return isSoftDisabled() ? 0 : m_joystick.map(Joystick::getY).orElse(0.0);
    }

    public double getRotation() {
        if (Constants.DISABLE_SWERVE) return 0;
        return isSoftDisabled() ?
        0 :
        (m_joystick.map(Joystick::getTrigger).orElse(false)
            ? m_joystick.map(Joystick::getZ).orElse(0.0)
            : 0
        );
    }

    // endregion movement

    // region utils

    private BooleanButton getFlightButton(int id) {
        return new FlightButton(id);
    }

    private BooleanButton getXBox(Function<XboxController, Boolean> getter) {
        return new XboxButton(getter);
    }

    private enum POVState {
        UP(true, false, false, false),
        UP_RIGHT(true, false, true, false),
        RIGHT(false, false, true, false),
        DOWN_RIGHT(false, true, true, false),
        DOWN(false, true, false, false),
        DOWN_LEFT(false, true, false, true),
        LEFT(false, false, false, true),
        UP_LEFT(true, false, false, true),
        NONE(false, false, false, false)
        ;
        public final boolean up;
        public final boolean down;
        public final boolean right;
        public final boolean left;

        POVState(boolean up, boolean down, boolean right, boolean left) {
            this.up = up;
            this.down = down;
            this.right = right;
            this.left = left;
        }

        public static POVState fromAngle(int angle) {
            if (angle < 0) {
                return POVState.NONE;
            }
            return values()[angle / 45];
        }
    }

    private BooleanButton getXBoxPOV(Function<POVState, Boolean> getter) {
        return new XboxButton(x -> getter.apply(POVState.fromAngle(x.getPOV())));
    }

    private DoubleControl getXBoxDouble(Function<XboxController, Double> getter) {
        return new XboxControl(getter);
    }

    @FunctionalInterface
    public interface BooleanButton {
        public static BooleanButton never = () -> false; 

        boolean get();

        default Trigger getTrigger() {
            return new Trigger(this::get);
        }
    }

    @FunctionalInterface
    public interface DoubleControl {
        double get();
    }

    private class FlightButton implements BooleanButton {
        private final int m_id;

        private FlightButton(int id) {
            this.m_id = id;
        }

        @Override
        public boolean get() {
            return isSoftDisabled()
                ? false
                : m_joystick.map(j -> j.getRawButton(m_id)).orElse(false);
        }
    }

    private class XboxButton implements BooleanButton {
        private final Function<XboxController, Boolean> m_getter;

        private XboxButton(Function<XboxController, Boolean> getter) {
            this.m_getter = getter;
        }

        @Override
        public boolean get() {
            return isSoftDisabled()
                ? false
                : m_xbox.map(m_getter).orElse(false);
        }
    }

    private class XboxControl implements DoubleControl {
        private final Function<XboxController, Double> m_getter;

        private XboxControl(Function<XboxController, Double> getter) {
            this.m_getter = getter;
        }

        @Override
        public double get() {
            return isSoftDisabled()
                ? 0.0
                : m_xbox.map(m_getter).orElse(0.0);
        }
    }

    // endregion utils

    // Preset buttons
    public final BooleanButton
        PRESET_REEF_LEFT_L4 = getFlightButton(7),
        PRESET_REEF_RIGHT_L4 = getFlightButton(8),
        PRESET_REEF_LEFT_L3 = getFlightButton(9),
        PRESET_REEF_RIGHT_L3 = getFlightButton(10),
        PRESET_REEF_LEFT_L2 = getFlightButton(11),
        PRESET_REEF_RIGHT_L2 = getFlightButton(12)
    ;
    public final BooleanButton
        PRESET_REEF_L1 = getXBox(XboxController::getBButton),
        PRESET_INTAKE = getXBox(XboxController::getYButton),
        PRESET_REEF_ALGAE = getXBox(XboxController::getXButton) // TODO
    ;

    // Aux drive buttons
    public final BooleanButton
        X_BRAKE = getFlightButton(2),
        ZERO_GYRO = getFlightButton(3),
        ZERO_GYRO_180 = getFlightButton(5),
        RESET_ELEVATOR = getFlightButton(4)
    ;

    // Climber buttons
    /*public final BooleanButton
        CLIMB_UP = getXBox(XboxController::getYButton),
        CLIMB_DOWN = getXBox(XboxController::getAButton),
        CLIMB_LOCK = getXBox(XboxController::getBButton),
        CLIMB_UNLOCK = getXBox(XboxController::getXButton)
    ;*/
    public final BooleanButton
        CLIMB_UP = BooleanButton.never,
        CLIMB_DOWN = BooleanButton.never,
        CLIMB_LOCK = BooleanButton.never,
        CLIMB_UNLOCK = BooleanButton.never
    ;

    // Intake/drop buttons
    public final BooleanButton
        /** Intake coral */
        INTAKE_CORAL = getXBox(x -> x.getLeftTriggerAxis() > 0.5),
        /** Intake algae */
        INTAKE_ALGAE = getXBox(XboxController::getLeftBumperButton),
        /** Simply drop held gamepiece, without additional sophistication */
        DROP_GAMEPIECE = getXBox(x -> x.getRightTriggerAxis() > 0.5),
        DROP_SLOW = getXBox(XboxController::getRightBumperButton)
    ;

    // Wrist
    public final DoubleControl
        WRIST_CONTROL = getXBoxDouble(x -> -x.getRightY())
    ;

    // Elevator
    public final DoubleControl
        ELEVATOR_CONTROL = getXBoxDouble(x -> -x.getLeftY())
    ;
    public final BooleanButton
        SYSID_SAFETY = BooleanButton.never
    ;

    public final BooleanButton
        CALIB_ELEVATOR_UP = getXBoxPOV(p -> p.up),
        CALIB_ELEVATOR_DOWN = getXBoxPOV(p -> p.down),
        CALIB_WRIST_UP = getXBoxPOV(p -> p.right),
        CALIB_WRIST_DOWN = getXBoxPOV(p -> p.left)
    ;

    // region soft disable

    private void softDisable() {
        SmartDashboard.putString("controller_mode", "Soft Disabled");
        this.m_soft_disabled = true;
    }

    private void softEnable() {
        SmartDashboard.putString("controller_mode", "Soft Enabled");
        this.m_soft_disabled = false;
    }

    public boolean isSoftDisabled() {
        if (this.m_auto_swap_mode && this.m_joystick.isPresent()) {
            double throttle = this.m_joystick.get().getThrottle();
            SmartDashboard.putNumber("twist", throttle);
            if (throttle < -0.75) {
                this.softEnable();
            } else if (throttle > 0.75) {
                this.softDisable();
            }
        }
        return this.m_soft_disabled;
    }

    // endregion soft disable

    public static BooleanSupplier anyAbove(double deadband, DoubleControl... inputs) {
        return () -> {
            for (DoubleControl input : inputs) {
                if (Math.abs(input.get()) > deadband) {
                    return true;
                }
            }
            return false;
        };
    }
}
