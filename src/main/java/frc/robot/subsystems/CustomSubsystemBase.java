package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CustomSubsystemBase extends SubsystemBase {
    public void log(String msg) {
        System.out.println(new StringBuilder(3 + msg.length() + getSubsystem().length())
        .append('[')
        .append(getSubsystem())
        .append(']')
        .append(' ')
        .append(msg));
    }

    private String k(String key) {
        return getSubsystem() + "/" + key;
    }

    protected void ntPutBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(k(key), value);
    }

    protected boolean ntGetBoolean(String key, boolean defaultValue) {
        return SmartDashboard.getBoolean(k(key), defaultValue);
    }

    protected void ntPutString(String key, String value) {
        SmartDashboard.putString(k(key), value);
    }

    protected String ntGetString(String key, String defaultValue) {
        return SmartDashboard.getString(k(key), defaultValue);
    }

    protected void ntPutNumber(String key, double value) {
        SmartDashboard.putNumber(k(key), value);
    }

    protected double ntGetNumber(String key, double defaultValue) {
        return SmartDashboard.getNumber(k(key), defaultValue);
    }
}
