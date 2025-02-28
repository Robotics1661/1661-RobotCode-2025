package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimitedTalonFX;

public class CustomSubsystemBase extends SubsystemBase {
    // Control request pool (for GC efficiency)

    public static final ControlRequest
        STATIC_BRAKE = new StaticBrake(),
        COAST_OUT = new CoastOut()
    ;

    private final Object NULL_KEY = new Object();
    private final Map<Object, Map<Class<? extends ControlRequest>, ControlRequest>> pooledRequests = new HashMap<>();

    protected <T extends ControlRequest> T getPooledRequest(Object key, Class<T> clazz, Supplier<T> create, Consumer<T> update) {
        if (key == null) {
            key = NULL_KEY;
        } else if (key instanceof TalonFX tfx) {
            key = tfx.getDeviceID();
        } else if (key instanceof LimitedTalonFX ltfx) {
            key = ltfx.m_wrapped.getDeviceID();
        }
        var pool = pooledRequests.computeIfAbsent(key, $ -> new HashMap<>());

        if (pool.containsKey(clazz)) {
            @SuppressWarnings("unchecked")
            T request = (T) pool.get(clazz);
            update.accept(request);
            return request;
        } else {
            T request = create.get();
            pool.put(clazz, request);
            return request;
        }
    }

    protected DutyCycleOut getPooledDutyCycleOut(Object key, double Output) {
        return getPooledRequest(key, DutyCycleOut.class, () -> new DutyCycleOut(Output), dco -> dco.withOutput(Output));
    }

    protected MotionMagicVelocityVoltage getPooledMotionMagicVelocityVoltage(Object key, AngularVelocity Velocity) {
        return getPooledRequest(key, MotionMagicVelocityVoltage.class, () -> new MotionMagicVelocityVoltage(Velocity), mmvv -> mmvv.withVelocity(Velocity));
    }

    protected VoltageOut getPooledVoltageOut(Object key, Voltage Output) {
        return getPooledRequest(key, VoltageOut.class, () -> new VoltageOut(Output), vo -> vo.withOutput(Output));
    }

    protected PositionVoltage getPooledPositionVoltage(Object key, Angle Position) {
        return getPooledRequest(key, PositionVoltage.class, () -> new PositionVoltage(Position), pv -> pv.withPosition(Position));
    }

    protected MotionMagicVoltage getPooledMotionMagicVoltage(Object key, Angle Position) {
        return getPooledRequest(key, MotionMagicVoltage.class, () -> new MotionMagicVoltage(Position), mmv -> mmv.withPosition(Position));
    }

    // Logging

    public void log(String msg) {
        System.out.println(new StringBuilder(3 + msg.length() + getSubsystem().length())
        .append('[')
        .append(getSubsystem())
        .append(']')
        .append(' ')
        .append(msg));
    }

    // NetworkTables

    private String k(String key) {
        return getSubsystem() + "/" + key;
    }

    protected void ntPutData(Consumer<SendableBuilder> init) {
        SmartDashboard.putData(getSubsystem(), new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType(getSubsystem());
                init.accept(builder);
            }
        });
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
