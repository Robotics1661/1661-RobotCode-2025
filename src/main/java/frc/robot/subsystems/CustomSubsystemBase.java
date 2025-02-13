package frc.robot.subsystems;

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
}
