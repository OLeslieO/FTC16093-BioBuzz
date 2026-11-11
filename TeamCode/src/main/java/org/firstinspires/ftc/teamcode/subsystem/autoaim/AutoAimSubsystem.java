package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * 云台自动瞄准（支持边走边射）。执行机构是两个 Axon MAX MK2 CR 舵机，
 * 角度由外置 REV Through Bore 编码器读取。：根据机器人在场地上的 Pedro 位姿和速度，把云台转向
 * {@link Alliance#getAimPose()}（即 {@code FieldConstants} 里的瞄准点）。
 *
 * <p>每个循环调用一次 {@link #update()}（或用 {@link #periodic()} 交给 Ivy）。
 * 位姿和速度由构造时传入的 supplier 提供，例如 {@code drive::getPose, drive::getVelocity}。
 * 速度必须是场地坐标系（Pedro 的 {@code follower.velocity()} 就是）。
 *
 * <p>每帧的计算流程：
 * <ol>
 *   <li>读云台角度 / 角速度（外置 Through Bore 编码器）</li>
 *   <li>底盘速度 / 加速度滤波；司机松摇杆时用刹车减速度预测（{@link ChassisMotionPredictor}）</li>
 *   <li>预测"出射时刻"的位姿：往前推 定位延迟 + 机械开火延迟（边走边射关闭时只推定位延迟）</li>
 *   <li>边走边射：虚拟目标 = 真实目标 − 出射点速度 × 滞空时间，滞空时间按距离查表并迭代</li>
 *   <li>云台目标角 = 指向虚拟目标的场地角 − 出射时 heading，在机械范围内选最近的等价角度</li>
 *   <li>速度前馈 = 视线角速度 − 底盘角速度：底盘怎么动，云台就提前反向补偿</li>
 *   <li>位置环输出目标角速度，再经 kS / kV / kA 前馈 + 速度环换算成功率，同时发给两个 CR 舵机</li>
 * </ol>
 *
 * <p>需要你提供的实机参数（都在 {@link AutoAimConfig}，Panels 可调）：
 * <ul>
 *   <li>两个 CR 舵机名和方向、编码器端口名和方向、减速比、零位、机械限位、旋转轴偏移</li>
 *   <li>舵机特性 kS / kV / KP_VEL（用 TurretServoTest 整定，换算成 rad/s）</li>
 *   <li>刹车减速度 FORWARD / LATERAL_BRAKE_DECEL</li>
 *   <li>机械开火延迟 SHOOT_DELAY、滞空时间表 FLIGHT_DIST_IN / FLIGHT_TIME_SEC</li>
 * </ul>
 */
public class AutoAimSubsystem {

    private final CRServo[] servos;
    private final DcMotorEx encoder;
    private final Supplier<Pose> poseSupplier;
    private final Supplier<Velocity> velocitySupplier;
    private final Telemetry telemetry;
    private final ChassisMotionPredictor predictor = new ChassisMotionPredictor();

    private boolean enabled = false;
    private boolean shootOnTheMove = false;
    private boolean braking = false;

    private long lastTimeNanos = 0;
    private boolean hasHistory = false;
    private double lastVelocityCommand = 0.0;
    private double filteredAccelCommand = 0.0;

    private double filteredVelocity = 0.0;

    private TurretAimCalculator.ShotSolution shot = null;
    private double fieldAngle = 0.0;
    private double targetAngle = 0.0;
    private double error = 0.0;
    private double feedforwardVelocity = 0.0;
    private double velocityCommand = 0.0;
    private double measuredVelocity = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private boolean reachable = true;
    private boolean locked = false;

    /**
     * @param velocitySupplier 场地坐标系速度（in/s，rad/s），例如 {@code follower::velocity}
     * @param resetEncoder     true = 把当前云台位置当作零点；
     *                         自动阶段之后进手动阶段时传 false，沿用自动阶段的编码器读数
     */
    public AutoAimSubsystem(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            Supplier<Pose> poseSupplier,
            Supplier<Velocity> velocitySupplier,
            boolean resetEncoder
    ) {
        this.telemetry = Objects.requireNonNull(telemetry);
        this.poseSupplier = Objects.requireNonNull(poseSupplier);
        this.velocitySupplier = Objects.requireNonNull(velocitySupplier);

        servos = new CRServo[]{
                hardwareMap.get(CRServo.class, AutoAimConfig.SERVO_NAME_0),
                hardwareMap.get(CRServo.class, AutoAimConfig.SERVO_NAME_1)
        };
        applyServoDirections();

        // Through Bore 插在电机端口上，只读编码器，不给这个端口输出功率
        encoder = hardwareMap.get(DcMotorEx.class, AutoAimConfig.ENCODER_NAME);
        if (resetEncoder) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0.0);
    }

    // ---------------------------------------------------------------
    // 开关 / 输入
    // ---------------------------------------------------------------

    public void enable() {
        enabled = true;
        resetFilters();
    }

    /** 停止瞄准并让云台停下。 */
    public void disable() {
        enabled = false;
        locked = false;
        power = 0.0;
        setServoPower(0.0);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** 开启后计算边走边射的提前量；关闭时直接瞄真实目标。 */
    public void setShootOnTheMove(boolean shootOnTheMove) {
        this.shootOnTheMove = shootOnTheMove;
    }

    public boolean isShootOnTheMove() {
        return shootOnTheMove;
    }

    /**
     * 告诉子系统底盘正在刹车（例如手动阶段摇杆回中、自动阶段路径结束），
     * 用来启用刹车预测。每个循环更新一次。
     */
    public void setBraking(boolean braking) {
        this.braking = braking;
    }

    // ---------------------------------------------------------------
    // 状态
    // ---------------------------------------------------------------

    /** 云台当前相对机器人正前方的角度（弧度）。 */
    public double getTurretAngle() {
        return encoderSign() * ticksToRadians(encoder.getCurrentPosition())
                + Math.toRadians(AutoAimConfig.ZERO_OFFSET_DEG);
    }

    /** 云台当前角速度（rad/s）。 */
    public double getTurretVelocity() {
        return encoderSign() * ticksToRadians(encoder.getVelocity());
    }

    /**
     * 是否可以开火：目标在机械范围内、角度误差在容差内、云台转速跟上了底盘、底盘速度不超限。
     */
    public boolean isLocked() {
        return enabled && locked;
    }

    /**
     * 出射点到（虚拟）目标的距离（英寸）。射速 / 俯仰请按这个距离查表，
     * 边走边射时它已经包含了提前量。
     */
    public double getShotDistance() {
        return shot != null ? shot.distance : 0.0;
    }

    /** 当前解算出的滞空时间（秒）。 */
    public double getFlightTime() {
        return shot != null ? shot.flightTime : 0.0;
    }

    /** 云台实际瞄准的场地坐标（边走边射时是虚拟目标）。 */
    public Pose getVirtualTarget() {
        return shot != null
                ? new Pose(shot.virtualX, shot.virtualY, 0.0)
                : Alliance.get().getAimPose();
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
        applyServoDirections();

        Pose robot = poseSupplier.get();
        predictor.update(robot, velocitySupplier.get(), dt);

        // 两个时刻：现在（只补偿定位延迟）用于速度前馈；出射时刻用于目标角
        ChassisMotionPredictor.Prediction present =
                predictor.predict(robot, AutoAimConfig.POSE_LATENCY, braking);
        double releaseHorizon = AutoAimConfig.POSE_LATENCY
                + (shootOnTheMove ? AutoAimConfig.SHOOT_DELAY : 0.0);
        ChassisMotionPredictor.Prediction release =
                predictor.predict(robot, releaseHorizon, braking);

        Pose aim = Alliance.get().getAimPose();
        Pose releasePivot = pivotOf(release);
        double[] releasePivotVel = pivotVelocity(release, releasePivot);

        double inherit = shootOnTheMove ? AutoAimConfig.VELOCITY_INHERITANCE : 0.0;
        shot = TurretAimCalculator.solveMovingShot(
                releasePivot.x(), releasePivot.y(),
                inherit * releasePivotVel[0], inherit * releasePivotVel[1],
                aim.x(), aim.y(),
                AutoAimConfig.FLIGHT_DIST_IN, AutoAimConfig.FLIGHT_TIME_SEC,
                AutoAimConfig.LEAD_ITERATIONS
        );

        fieldAngle = Math.atan2(shot.virtualY - releasePivot.y(), shot.virtualX - releasePivot.x())
                + Math.toRadians(AutoAimConfig.YAW_TRIM_DEG);
        double desired = fieldAngle - release.heading;

        double current = getTurretAngle();
        filteredVelocity += AutoAimConfig.VEL_FILTER_ALPHA * (getTurretVelocity() - filteredVelocity);
        measuredVelocity = filteredVelocity;
        targetAngle = TurretAimCalculator.chooseTurretAngle(
                desired, current,
                Math.toRadians(AutoAimConfig.MIN_ANGLE_DEG),
                Math.toRadians(AutoAimConfig.MAX_ANGLE_DEG));
        reachable = Math.abs(Angle.normalizeSigned(desired - targetAngle)) < 1e-6;
        error = targetAngle - current;

        // 云台相对角的变化率 = 视线角速度 − 底盘角速度。目标在死区里时不前馈，免得往限位上顶
        Pose presentPivot = pivotOf(present);
        double[] presentPivotVel = pivotVelocity(present, presentPivot);
        // 底盘静止时，定位速度噪声会让前馈在 0 附近来回变号；软死区把这种噪声滤掉
        feedforwardVelocity = reachable
                ? softDeadband(TurretAimCalculator.lineOfSightRate(
                        presentPivot.x(), presentPivot.y(),
                        presentPivotVel[0], presentPivotVel[1],
                        shot.virtualX, shot.virtualY)
                - present.omega, AutoAimConfig.FF_DEADBAND)
                : 0.0;

        // 软死区：从死区边缘连续地长出来，不会在边缘处突然跳一个速度指令
        double positionVelocity = AutoAimConfig.KP_POS
                * softDeadband(error, Math.toRadians(AutoAimConfig.DEADBAND_DEG));
        velocityCommand = clip(positionVelocity + feedforwardVelocity,
                -AutoAimConfig.MAX_TURRET_VEL, AutoAimConfig.MAX_TURRET_VEL);

        double rawAccelCommand = dt > 0.0 ? (velocityCommand - lastVelocityCommand) / dt : 0.0;
        filteredAccelCommand += AutoAimConfig.CMD_ACCEL_FILTER_ALPHA
                * (rawAccelCommand - filteredAccelCommand);
        lastVelocityCommand = velocityCommand;
        hasHistory = true;

        // kS 随速度指令线性长出来（到 KS_RAMP_VEL 时满额），指令在 0 附近抖动时不会让功率来回跳 ±kS
        double staticFriction = AutoAimConfig.kS
                * clip(velocityCommand / AutoAimConfig.KS_RAMP_VEL, -1.0, 1.0);
        double rawPower = staticFriction
                + AutoAimConfig.kV * velocityCommand
                + AutoAimConfig.kA * filteredAccelCommand
                + AutoAimConfig.KP_VEL * (velocityCommand - measuredVelocity);

        power = applyLimits(clip(rawPower, -AutoAimConfig.MAX_POWER, AutoAimConfig.MAX_POWER), current);
        setServoPower(power);

        tolerance = TurretAimCalculator.lockTolerance(
                shot.distance,
                AutoAimConfig.AIM_HALF_WIDTH_IN,
                Math.toRadians(AutoAimConfig.MIN_LOCK_TOLERANCE_DEG),
                Math.toRadians(AutoAimConfig.MAX_LOCK_TOLERANCE_DEG));
        double maxSpeed = shootOnTheMove
                ? AutoAimConfig.MAX_SHOOT_SPEED
                : AutoAimConfig.MAX_STATIONARY_SHOOT_SPEED;
        locked = reachable
                && Math.abs(error) <= tolerance
                && Math.abs(measuredVelocity - feedforwardVelocity)
                        <= Math.toRadians(AutoAimConfig.LOCK_MAX_RATE_ERROR_DEG)
                && predictor.speed() <= maxSpeed;
    }

    private void resetFilters() {
        hasHistory = false;
        lastVelocityCommand = 0.0;
        filteredAccelCommand = 0.0;
        predictor.reset();
        filteredVelocity = getTurretVelocity();
    }

    // ---------------------------------------------------------------
    // 几何辅助
    // ---------------------------------------------------------------

    private static Pose pivotOf(ChassisMotionPredictor.Prediction state) {
        return TurretAimCalculator.pivotPose(
                new Pose(state.x, state.y, state.heading),
                AutoAimConfig.PIVOT_OFFSET_FORWARD_IN,
                AutoAimConfig.PIVOT_OFFSET_LEFT_IN);
    }

    /** 旋转轴的场地速度 = 底盘速度 + ω × 偏移（偏移不在中心时，原地转也会让旋转轴移动）。 */
    private static double[] pivotVelocity(ChassisMotionPredictor.Prediction state, Pose pivot) {
        double rx = pivot.x() - state.x;
        double ry = pivot.y() - state.y;
        return new double[]{
                state.vx - state.omega * ry,
                state.vy + state.omega * rx
        };
    }

    /** 软死区：|x| ≤ band 时为 0，之外减去 band，保持符号且连续。 */
    private static double softDeadband(double x, double band) {
        return Math.abs(x) <= band ? 0.0 : x - Math.copySign(band, x);
    }

    private static double ticksToRadians(double ticks) {
        return ticks / AutoAimConfig.ticksPerTurretRev() * 2.0 * Math.PI;
    }

    // ---------------------------------------------------------------
    // 舵机 / 编码器
    // ---------------------------------------------------------------

    private static double encoderSign() {
        return AutoAimConfig.ENCODER_REVERSED ? -1.0 : 1.0;
    }

    /** 方向每圈重设，这样 Panels 里实时翻转 SERVO_REVERSED_x 立刻生效。 */
    private void applyServoDirections() {
        servos[0].setDirection(AutoAimConfig.SERVO_REVERSED_0
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servos[1].setDirection(AutoAimConfig.SERVO_REVERSED_1
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void setServoPower(double p) {
        for (CRServo servo : servos) {
            servo.setPower(p);
        }
    }

    // ---------------------------------------------------------------
    // 限位 / 遥测
    // ---------------------------------------------------------------

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
        String state;
        if (!enabled) {
            state = "OFF";
        } else if (!reachable) {
            state = "OUT OF RANGE";
        } else {
            state = locked ? "LOCKED" : "TRACKING";
        }
        telemetry.addData("AutoAim", state);
        telemetry.addData("Aim Mode", shootOnTheMove ? "SHOOT ON THE MOVE" : "STATIONARY");
        telemetry.addData("Aim Braking", braking);
        telemetry.addData("Aim Field Angle", "%.1f°", Math.toDegrees(fieldAngle));
        telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(getTurretAngle()));
        telemetry.addData("Turret Target", "%.1f°", Math.toDegrees(targetAngle));
        telemetry.addData("Aim Error", "%.2f° (tol %.2f°)", Math.toDegrees(error), Math.toDegrees(tolerance));
        telemetry.addData("Turret Vel", "%.1f°/s (ff %.1f°/s)",
                Math.toDegrees(measuredVelocity), Math.toDegrees(feedforwardVelocity));
        telemetry.addData("Turret Vel Cmd", "%.1f°/s", Math.toDegrees(velocityCommand));
        telemetry.addData("Turret Power", "%.3f", power);
        telemetry.addData("Chassis Speed", "%.1f in/s", predictor.speed());
        if (shot != null) {
            telemetry.addData("Shot Distance", "%.1f in", shot.distance);
            telemetry.addData("Flight Time", "%.3f s", shot.flightTime);
            telemetry.addData("Virtual Target", "(%.1f, %.1f)", shot.virtualX, shot.virtualY);
        }
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
