package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * 底盘运动估计：滤波速度 / 加速度，并预测未来一小段时间后的位置和速度。
 *
 * <p>速度在车体坐标系（前 / 左）里滤波，这样底盘原地转向时滤波器不会把速度方向"拖"歪；
 * 刹车减速度也是按车体前 / 横两个方向分别实测的。
 *
 * <p>刹车预测：司机松开摇杆时底盘会按近似恒定的减速度停下。此时用
 * "实测加速度和刹车减速度的加权"代替纯实测加速度（实测值有滞后），
 * 并且预测速度不允许越过 0 反向。
 */
public class ChassisMotionPredictor {

    /** 预测结果，全部是场地坐标系。 */
    static final class Prediction {
        final double x;
        final double y;
        final double heading;
        final double vx;
        final double vy;
        final double omega;

        Prediction(double x, double y, double heading, double vx, double vy, double omega) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }

    private boolean initialized = false;

    private double forward = 0.0;
    private double lateral = 0.0;
    private double forwardAccel = 0.0;
    private double lateralAccel = 0.0;
    private double omega = 0.0;

    void reset() {
        initialized = false;
    }

    /**
     * 喂入一帧数据。
     *
     * @param fieldVelocity 场地坐标系速度（Pedro / Pinpoint 给的就是场地系），in/s 与 rad/s
     * @param dt            距上一帧的时间（秒）
     */
    void update(Pose pose, Velocity fieldVelocity, double dt) {
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        double rawForward = fieldVelocity.vx * cos + fieldVelocity.vy * sin;
        double rawLateral = -fieldVelocity.vx * sin + fieldVelocity.vy * cos;

        if (!initialized || dt <= 0.0) {
            forward = rawForward;
            lateral = rawLateral;
            forwardAccel = 0.0;
            lateralAccel = 0.0;
            omega = fieldVelocity.omega;
            initialized = true;
            return;
        }

        double lastForward = forward;
        double lastLateral = lateral;
        forward += AutoAimConfig.CHASSIS_VEL_FILTER_ALPHA * (rawForward - forward);
        lateral += AutoAimConfig.CHASSIS_VEL_FILTER_ALPHA * (rawLateral - lateral);

        double accelAlpha = AutoAimConfig.CHASSIS_ACCEL_FILTER_ALPHA;
        forwardAccel += accelAlpha * ((forward - lastForward) / dt - forwardAccel);
        lateralAccel += accelAlpha * ((lateral - lastLateral) / dt - lateralAccel);

        omega += AutoAimConfig.OMEGA_FILTER_ALPHA * (fieldVelocity.omega - omega);
    }

    /** 当前滤波后的场地系速度。 */
    Prediction current(Pose pose) {
        return predict(pose, 0.0, false);
    }

    /**
     * 预测 {@code horizon} 秒之后的状态。
     *
     * <p>每个车体轴按匀加速积分；horizon 很短（几十毫秒），区间内的转向忽略不计，
     * 只把最终 heading 往前推 ω·horizon。
     */
    Prediction predict(Pose pose, double horizon, boolean braking) {
        double predictedForwardAccel = predictAxisAccel(
                forward, forwardAccel, braking, AutoAimConfig.FORWARD_BRAKE_DECEL);
        double predictedLateralAccel = predictAxisAccel(
                lateral, lateralAccel, braking, AutoAimConfig.LATERAL_BRAKE_DECEL);

        double[] forwardMotion = integrateAxis(forward, predictedForwardAccel, horizon, braking);
        double[] lateralMotion = integrateAxis(lateral, predictedLateralAccel, horizon, braking);

        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        return new Prediction(
                pose.x() + forwardMotion[0] * cos - lateralMotion[0] * sin,
                pose.y() + forwardMotion[0] * sin + lateralMotion[0] * cos,
                pose.heading() + omega * horizon,
                forwardMotion[1] * cos - lateralMotion[1] * sin,
                forwardMotion[1] * sin + lateralMotion[1] * cos,
                omega
        );
    }

    /** 滤波后的车速（in/s）。 */
    double speed() {
        return Math.hypot(forward, lateral);
    }

    private static double predictAxisAccel(
            double velocity, double measuredAccel, boolean braking, double brakeDecel) {
        if (!braking || Math.abs(velocity) < 1e-4) {
            return measuredAccel;
        }
        double brakeAccel = -Math.signum(velocity) * brakeDecel;
        double w = AutoAimConfig.BRAKE_PREDICTION_WEIGHT;
        return measuredAccel * (1.0 - w) + brakeAccel * w;
    }

    /**
     * 单轴匀加速积分，返回 {位移, 末速度}。
     * 刹车时速度只会减到 0 就停住，不会反向：先减速到停，剩下的时间原地不动。
     */
    static double[] integrateAxis(double v0, double accel, double horizon, boolean braking) {
        double vEnd = v0 + accel * horizon;
        boolean crossesZero = (v0 > 0.0 && vEnd < 0.0) || (v0 < 0.0 && vEnd > 0.0);
        if (braking && crossesZero) {
            double stopTime = -v0 / accel;
            return new double[]{0.5 * v0 * stopTime, 0.0};
        }
        return new double[]{v0 * horizon + 0.5 * accel * horizon * horizon, vEnd};
    }
}
