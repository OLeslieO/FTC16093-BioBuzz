package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * 云台自动瞄准：根据机器人在场地上的 Pedro 位姿，把云台转向
 * {@link Alliance#getAimPose()}（即 {@code FieldConstants} 里的瞄准点）。
 *
 * <p>每个循环调用一次 {@link #update()}（或用 {@link #periodic()} 交给 Ivy）。
 * 位姿由构造时传入的 supplier 提供，例如 {@code drive::getPose}。
 *
 * <p>计算流程：
 * <ol>
 *   <li>云台旋转轴的场地位置 = 机器人位置 + 偏移（{@link AutoAimConfig#PIVOT_OFFSET_FORWARD_IN} 等）</li>
 *   <li>场地绝对角 = atan2(瞄准点 - 旋转轴)</li>
 *   <li>云台相对角 = 场地绝对角 - 机器人 heading，在机械范围内选最近的等价角度</li>
 *   <li>PD + 底盘旋转前馈输出电机功率</li>
 * </ol>
 *
 * <p>需要你提供的实机参数（都在 {@link AutoAimConfig}，Panels 可调）：
 * <ul>
 *   <li>云台电机名、转向是否反了、整圈 ticks（电机 ticks × 总减速比）</li>
 *   <li>编码器为 0 时云台相对车头的角度；云台左右机械限位角度</li>
 *   <li>云台旋转轴相对机器人中心（Pedro 定位点）的前 / 左偏移</li>
 *   <li>控制增益 kP / kD / kV_OMEGA / kS，以及最大功率</li>
 * </ul>
 */
public class AutoAimSubsystem {

    private final DcMotorEx motor;
    private final Supplier<Pose> poseSupplier;
    private final Telemetry telemetry;

    private boolean enabled = false;

    private long lastTimeNanos = 0;
    private double lastError = 0.0;
    private double lastHeading = 0.0;
    private boolean hasHistory = false;
    private double filteredErrorRate = 0.0;
    private double filteredOmega = 0.0;

    private double fieldAngle = 0.0;
    private double targetAngle = 0.0;
    private double error = 0.0;
    private double distanceToAim = 0.0;
    private boolean locked = false;

    /**
     * @param resetEncoder true = 把当前云台位置当作零点；
     *                     自动阶段之后进手动阶段时传 false，沿用自动阶段的编码器读数
     */
    public AutoAimSubsystem(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            Supplier<Pose> poseSupplier,
            boolean resetEncoder
    ) {
        this.telemetry = Objects.requireNonNull(telemetry);
        this.poseSupplier = Objects.requireNonNull(poseSupplier);

        motor = hardwareMap.get(DcMotorEx.class, AutoAimConfig.MOTOR_NAME);
        motor.setDirection(AutoAimConfig.MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---------------------------------------------------------------
    // 开关
    // ---------------------------------------------------------------

    public void enable() {
        enabled = true;
        hasHistory = false;
    }

    /** 停止瞄准并让云台停下。 */
    public void disable() {
        enabled = false;
        locked = false;
        motor.setPower(0.0);
    }

    public boolean isEnabled() {
        return enabled;
    }

    // ---------------------------------------------------------------
    // 状态
    // ---------------------------------------------------------------

    /** 云台当前相对机器人正前方的角度（弧度）。 */
    public double getTurretAngle() {
        return motor.getCurrentPosition() / AutoAimConfig.TICKS_PER_TURRET_REV * 2.0 * Math.PI
                + Math.toRadians(AutoAimConfig.ZERO_OFFSET_DEG);
    }

    /** 云台是否已对准瞄准点（误差小于 {@link AutoAimConfig#LOCK_TOLERANCE_DEG}）。 */
    public boolean isLocked() {
        return enabled && locked;
    }

    /** 云台旋转轴到瞄准点的距离（英寸），后续可用来查射速表。 */
    public double getDistanceToAim() {
        return distanceToAim;
    }

    // ---------------------------------------------------------------
    // 更新
    // ---------------------------------------------------------------

    /** 每个 OpMode 循环调用一次。 */
    public void update() {
        if (enabled) {
            runAim();
        }
        sendTelemetry();
    }

    /** Ivy 周期命令，只需调度一次。 */
    public Command periodic() {
        return infinite(this::update);
    }

    private void runAim() {
        long now = System.nanoTime();
        double dt = hasHistory ? Math.max((now - lastTimeNanos) / 1e9, 1e-3) : 0.0;
        lastTimeNanos = now;

        Pose robot = poseSupplier.get();
        Pose aim = Alliance.get().getAimPose();
        Pose pivot = TurretAimCalculator.pivotPose(
                robot,
                AutoAimConfig.PIVOT_OFFSET_FORWARD_IN,
                AutoAimConfig.PIVOT_OFFSET_LEFT_IN
        );

        fieldAngle = TurretAimCalculator.fieldAngleToAim(pivot, aim);
        distanceToAim = Math.hypot(aim.x() - pivot.x(), aim.y() - pivot.y());

        double current = getTurretAngle();
        double min = Math.toRadians(AutoAimConfig.MIN_ANGLE_DEG);
        double max = Math.toRadians(AutoAimConfig.MAX_ANGLE_DEG);
        targetAngle = TurretAimCalculator.chooseTurretAngle(
                fieldAngle - robot.heading(), current, min, max);
        error = targetAngle - current;

        // 底盘角速度和误差变化率都从相邻两帧估算，再做低通
        double alpha = AutoAimConfig.FILTER_ALPHA;
        if (hasHistory) {
            double omega = Angle.normalizeSigned(robot.heading() - lastHeading) / dt;
            filteredOmega += alpha * (omega - filteredOmega);
            double errorRate = (error - lastError) / dt;
            filteredErrorRate += alpha * (errorRate - filteredErrorRate);
        } else {
            filteredOmega = 0.0;
            filteredErrorRate = 0.0;
        }
        lastHeading = robot.heading();
        lastError = error;
        hasHistory = true;

        locked = Math.abs(error) <= Math.toRadians(AutoAimConfig.LOCK_TOLERANCE_DEG);

        double power = 0.0;
        if (Math.abs(error) > Math.toRadians(AutoAimConfig.DEADBAND_DEG)) {
            power = AutoAimConfig.kP * error
                    + AutoAimConfig.kD * filteredErrorRate
                    - AutoAimConfig.kV_OMEGA * filteredOmega
                    + Math.copySign(AutoAimConfig.kS, error);
        }
        motor.setPower(applyLimits(clip(power, -AutoAimConfig.MAX_POWER, AutoAimConfig.MAX_POWER), current));
    }

    /** 贴近机械限位时，不允许继续朝限位方向输出。 */
    private static double applyLimits(double power, double currentAngle) {
        double margin = Math.toRadians(AutoAimConfig.LIMIT_MARGIN_DEG);
        if (power > 0.0 && currentAngle >= Math.toRadians(AutoAimConfig.MAX_ANGLE_DEG) - margin) {
            return 0.0;
        }
        if (power < 0.0 && currentAngle <= Math.toRadians(AutoAimConfig.MIN_ANGLE_DEG) + margin) {
            return 0.0;
        }
        return power;
    }

    private void sendTelemetry() {
        telemetry.addData("AutoAim", enabled ? (locked ? "LOCKED" : "TRACKING") : "OFF");
        telemetry.addData("Aim Alliance", Alliance.get());
        telemetry.addData("Aim Field Angle", "%.1f°", Math.toDegrees(fieldAngle));
        telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(getTurretAngle()));
        telemetry.addData("Turret Target", "%.1f°", Math.toDegrees(targetAngle));
        telemetry.addData("Aim Error", "%.2f°", Math.toDegrees(error));
        telemetry.addData("Aim Distance", "%.1f in", distanceToAim);
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
