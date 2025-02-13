package frc.robot;

import java.util.Optional;
import java.util.function.Function;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoyXboxWrapper {
    private final Optional<XboxController> m_xbox;
    private final Optional<Joystick> m_joystick; //flightstick
    @SuppressWarnings("unused")
    private boolean m_soft_disabled;
    @SuppressWarnings("unused")
    private final boolean m_auto_swap_mode = true; // If true, soft enable if twist is on +

    public JoyXboxWrapper(int xboxPort, int joystickPort) {
        this(xboxPort, joystickPort, false);
    }

    public JoyXboxWrapper(int xboxPort, int joystickPort, boolean controllerMode) {
        if (Utils.isSimulation()) {
            m_xbox = Optional.empty();
            m_joystick = Optional.empty();
        } else {
            m_xbox = Optional.of(new XboxController(xboxPort));
            m_joystick = Optional.of(new Joystick(joystickPort));
        }
        m_soft_disabled = controllerMode;
    }

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

    private boolean getFlightButton(int id) {
        return isSoftDisabled()
            ? false
            : m_joystick.map(j -> j.getRawButton(id)).orElse(false);
    }

    private boolean getXBox(Function<XboxController, Boolean> function) {
        return isSoftDisabled()
            ? false
            : m_xbox.map(function).orElse(false);
    }

    public boolean getElevatorUpButton() {
        return getXBox(x -> x.getYButton());
    }

    public boolean getElevatorDownButton() {
        return getXBox(x -> x.getAButton());
    }

    public boolean getZeroButton() {
        return getFlightButton(7);
    }

    public boolean getBrakeButton() {
        return getFlightButton(9);
    }

    public boolean getZeroButton180() {
        return getFlightButton(10);
    }

    public void softDisable() {
        SmartDashboard.putString("controller_mode", "Soft Disabled");
        this.m_soft_disabled = true;
    }

    public void softEnable() {
        SmartDashboard.putString("controller_mode", "Soft Enabled");
        this.m_soft_disabled = false;
    }

    public boolean isSoftDisabled() {
        return false; // FIXME: temporary to allow joystick-less use
        /*if (this.m_auto_swap_mode) {
            SmartDashboard.putNumber("twist", this.m_joystick.getThrottle());
            if (this.m_joystick.getThrottle() < -0.75) {
                this.softEnable();
            } else if (this.m_joystick.getThrottle() > 0.75) {
                this.softDisable();
            }
        }
        return this.m_soft_disabled;*/
    }
}
