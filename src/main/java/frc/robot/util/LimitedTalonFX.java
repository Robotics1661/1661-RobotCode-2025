package frc.robot.util;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class LimitedTalonFX {
    private static final Set<Class<? extends ControlRequest>> SAFE_REQUESTS = Set.of(
        DutyCycleOut.class,
        PositionVoltage.class,
        VoltageOut.class,
        MotionMagicVoltage.class,
        MotionMagicVelocityVoltage.class,
        StaticBrake.class,
        CoastOut.class
    );

    public final TalonFX m_wrapped;
    private final BooleanSupplier m_forwardLimit;
    private final BooleanSupplier m_reverseLimit;

    private ControlRequest m_request;

    public LimitedTalonFX(TalonFX motor, BooleanSupplier forwardLimit, BooleanSupplier reverseLimit) {
        this.m_wrapped = motor;
        this.m_forwardLimit = forwardLimit;
        this.m_reverseLimit = reverseLimit;
        this.m_request = null;
    }

    /**
     * Update limit status of request, if applicable
     * @param request request to update
     * @param limitForward whether to prevent forward motion
     * @param limitReverse whether to prevent reverse motion
     * @return whether motor needs update
     */
    private static boolean updateRequest(ControlRequest request, boolean limitForward, boolean limitReverse) {
        if (request instanceof DutyCycleOut dco) {
            if (limitForward != dco.LimitForwardMotion || limitReverse != dco.LimitReverseMotion) {
                dco.withLimitForwardMotion(limitForward).withLimitReverseMotion(limitReverse);
                return true;
            }
        } else if (request instanceof PositionVoltage pv) {
            if (limitForward != pv.LimitForwardMotion || limitReverse != pv.LimitReverseMotion) {
                pv.withLimitForwardMotion(limitForward).withLimitReverseMotion(limitReverse);
                return true;
            }
        } else if (request instanceof VoltageOut vo) {
            if (limitForward != vo.LimitForwardMotion || limitReverse != vo.LimitReverseMotion) {
                vo.withLimitForwardMotion(limitForward).withLimitReverseMotion(limitReverse);
                return true;
            }
        } else if (request instanceof MotionMagicVoltage mmv) {
            if (limitForward != mmv.LimitForwardMotion || limitReverse != mmv.LimitReverseMotion) {
                mmv.withLimitForwardMotion(limitForward).withLimitReverseMotion(limitReverse);
                return true;
            }
        } else if (request instanceof MotionMagicVelocityVoltage mmvv) {
            if (limitForward != mmvv.LimitForwardMotion || limitReverse != mmvv.LimitReverseMotion) {
                mmvv.withLimitForwardMotion(limitForward).withLimitReverseMotion(limitReverse);
                return true;
            }
        }
        return false;
    }

    /**
     * MUST be called to update limits
     */
    public void periodic() {
        if (m_request != null && updateRequest(m_request, m_forwardLimit.getAsBoolean(), m_reverseLimit.getAsBoolean())) {
            m_wrapped.setControl(m_request);
        }
    }

    public <T extends ControlRequest> void setControl(T request) {
        if (!SAFE_REQUESTS.contains(request.getClass())) {
            m_wrapped.setControl(new CoastOut());
            throw new RuntimeException("Unknown control request, cannot apply safely: "+request);
        }
        updateRequest(request, m_forwardLimit.getAsBoolean(), m_reverseLimit.getAsBoolean());
        m_request = request;
        m_wrapped.setControl(m_request);
    }
}
