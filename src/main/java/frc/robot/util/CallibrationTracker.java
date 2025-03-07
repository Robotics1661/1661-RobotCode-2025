package frc.robot.util;

import static frc.robot.util.AutonomousDebugger.autoDbg;

import edu.wpi.first.wpilibj2.command.Command;

public class CallibrationTracker {
    private static boolean isCallibrated = false;

    public static void reset() {
        isCallibrated = false;
    }

    public static void markCallibrated() {
        isCallibrated = true;
    }

    public static class WaitForCallibrationCommand extends Command {
        private boolean log = false;

        public WaitForCallibrationCommand() {}

        @Override
        public void initialize() {
            if (!isFinished()) {
                autoDbg(">> Waiting for callibration");
                log = true;
            }
        }

        @Override
        public boolean isFinished() {
            return isCallibrated;
        }

        @Override
        public void end(boolean interrupted) {
            if (log) {
                autoDbg(">> Finished waiting for callibration");
            }
        }
    }
}
