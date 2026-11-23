package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Objects;
import java.util.function.Supplier;


/**
 * 自瞄云台。两个舵机工作在位置模式（舵机自己闭环），这里只算目标角并换算成舵机位置；
 * Through Bore 编码器用来读实际角度（判断锁定、可选的稳态误差修正）。
 *
 * <p>舵机位置 = CENTER_POS + 角度 / TURRET_DEG_PER_POS，用 TurretPositionServoTest MODE 4 标定。
 */
public class AutoAimSubsystem {

    private final ServoImplEx[] servos;
    private final DcMotorEx encoder;
    private final Supplier<Pose> poseSupplier;
    private final Supplier<Velocity> velocitySupplier;
    private final Telemetry telemetry;
    private final ChassisMotionPredictor predictor = new ChassisMotionPredictor();

    private boolean enabled = false;
    private boolean holding = false;
    private double holdAngle = 0.0;
    private boolean shootOnTheMove = false;
    private boolean braking = false;

    private long lastTimeNanos = 0;
    private boolean hasHistory = false;

    private double filteredVelocity = 0.0;

    private TurretAimCalculator.ShotSolution shot = null;
    private double fieldAngle = 0.0;
    private double targetAngle = 0.0;
    private double error = 0.0;
    private double feedforwardVelocity = 0.0;
    private double measuredVelocity = 0.0;
    /** 实际发给舵机的角度（rad）：目标 + 超前补偿 + 修正，经过限速。 */
    private double commandAngle = 0.0;
    /** 编码器积分修正量（rad）。 */
    private double trim = 0.0;
    private double servoPosition = Double.NaN;
    private boolean pwmEnabled = false;
    private double appliedPwmMin = Double.NaN;
    private double appliedPwmMax = Double.NaN;
    private double tolerance = 0.0;
    private boolean reachable = true;
    private boolean locked = false;


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

        // 硬件配置里这两个口要设成 Servo，不是 Continuous Rotation Servo
        servos = new ServoImplEx[]{
                hardwareMap.get(ServoImplEx.class, AutoAimConfig.SERVO_NAME_0),
                hardwareMap.get(ServoImplEx.class, AutoAimConfig.SERVO_NAME_1)
        };
        applyServoConfig();
        // 开瞄准之前不给 PWM，INIT 时云台不会突然转
        setPwmEnabled(false);

        encoder = hardwareMap.get(DcMotorEx.class, AutoAimConfig.ENCODER_NAME);
        if (resetEncoder) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0.0);
    }

    public void enable() {
        enabled = true;
        resetFilters();
    }

    public void disable() {
        enabled = false;
        holding = false;
        locked = false;
        setPwmEnabled(false);
    }

    public boolean isEnabled() {
        return enabled;
    }


    public void holdAngle(double angleRad) {
        if (!holding) {
            resetFilters();
        }
        holding = true;
        holdAngle = angleRad;
        locked = false;
        shot = null;
    }

    public void releaseHold() {
        if (!holding) {
            return;
        }
        holding = false;
        resetFilters();
        if (!enabled) {
            setPwmEnabled(false);
        }
    }

    public boolean isHolding() {
        return holding;
    }

    // ---- 调参用：最近一圈控制器的内部量 ----

    /** 最近一次发给舵机 0 的位置（0~1），没开 PWM 时为 NaN。 */
    public double getServoPosition() {
        return servoPosition;
    }

    /** 实际发给舵机的角度（rad）。 */
    public double getCommandAngle() {
        return commandAngle;
    }

    /** 编码器积分修正量（rad）。 */
    public double getTrim() {
        return trim;
    }

    /** 目标角 − 当前角（rad）。 */
    public double getError() {
        return error;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /** 滤波后的云台实测速度（rad/s）。 */
    public double getMeasuredVelocity() {
        return measuredVelocity;
    }

    public void setShootOnTheMove(boolean shootOnTheMove) {
        this.shootOnTheMove = shootOnTheMove;
    }

    public boolean isShootOnTheMove() {
        return shootOnTheMove;
    }


    public void setBraking(boolean braking) {
        this.braking = braking;
    }


    public double getTurretAngle() {
        return encoderSign() * ticksToRadians(encoder.getCurrentPosition())
                + Math.toRadians(AutoAimConfig.ZERO_OFFSET_DEG);
    }

    public double getTurretVelocity() {
        return encoderSign() * ticksToRadians(encoder.getVelocity());
    }


    public boolean isLocked() {
        return enabled && locked;
    }


    public double getShotDistance() {
        return shot != null ? shot.distance : 0.0;
    }

    public double getFlightTime() {
        return shot != null ? shot.flightTime : 0.0;
    }

    public Pose getVirtualTarget() {
        return shot != null
                ? new Pose(shot.virtualX, shot.virtualY, 0.0)
                : Alliance.get().getAimPose();
    }

    public void update() {
        if (holding) {
            runHold();
        } else if (enabled) {
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
        applyServoConfig();

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
                desired, current, minAngle(), maxAngle());
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

        driveTurret(current, dt);

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

    /** 固定角度模式：目标角 = holdAngle（夹在机械范围内），没有前馈。 */
    private void runHold() {
        long now = System.nanoTime();
        double dt = hasHistory ? Math.max((now - lastTimeNanos) / 1e9, 1e-3) : 0.0;
        lastTimeNanos = now;
        applyServoConfig();

        double current = getTurretAngle();
        filteredVelocity += AutoAimConfig.VEL_FILTER_ALPHA * (getTurretVelocity() - filteredVelocity);
        measuredVelocity = filteredVelocity;
        targetAngle = clip(holdAngle, minAngle(), maxAngle());
        reachable = true;
        error = targetAngle - current;
        feedforwardVelocity = 0.0;

        driveTurret(current, dt);
        locked = false;
    }

    /**
     * 目标角 → 舵机位置。舵机自己做位置闭环，这里只做三件事：
     * <ol>
     *   <li>超前补偿：目标在动时多给 feedforwardVelocity × SERVO_LEAD_TIME，抵消舵机跟随的滞后；</li>
     *   <li>可选的编码器积分修正（TRIM_KI），补齿隙 / 标定误差带来的稳态偏差；</li>
     *   <li>可选的限速（MAX_TURRET_VEL），舵机冲得太猛时打开。</li>
     * </ol>
     */
    private void driveTurret(double current, double dt) {
        double lo = minAngle();
        double hi = maxAngle();

        // 只在快到位、目标不怎么动时积分，避免运动过程中积出一大坨
        if (AutoAimConfig.TRIM_KI > 0.0
                && Math.abs(error) < Math.toRadians(AutoAimConfig.TRIM_WINDOW_DEG)
                && Math.abs(feedforwardVelocity) < 1e-6) {
            double trimError = softDeadband(error, Math.toRadians(AutoAimConfig.DEADBAND_DEG));
            double maxTrim = Math.toRadians(AutoAimConfig.TRIM_MAX_DEG);
            trim = clip(trim + AutoAimConfig.TRIM_KI * trimError * dt, -maxTrim, maxTrim);
        } else if (AutoAimConfig.TRIM_KI <= 0.0) {
            trim = 0.0;
        }

        double desiredCommand = clip(
                targetAngle + feedforwardVelocity * AutoAimConfig.SERVO_LEAD_TIME + trim, lo, hi);

        if (!hasHistory) {
            commandAngle = current;
        }
        if (AutoAimConfig.MAX_TURRET_VEL > 0.0 && dt > 0.0) {
            double step = AutoAimConfig.MAX_TURRET_VEL * dt;
            commandAngle += clip(desiredCommand - commandAngle, -step, step);
        } else {
            commandAngle = desiredCommand;
        }
        hasHistory = true;

        setServoAngle(commandAngle);
    }

    private void resetFilters() {
        hasHistory = false;
        trim = 0.0;
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

    /**
     * 编码器读数符号。Through Bore 和 transfer 电机共用 intake1 端口，
     * TransferSubsystem 会把这个电机设成 REVERSE，SDK 会连编码器读数一起取反，
     * 所以这里把电机方向抵消掉，只认 ENCODER_REVERSED。
     */
    private double encoderSign() {
        double motorDirSign = encoder.getDirection() == DcMotorSimple.Direction.REVERSE ? -1.0 : 1.0;
        return (AutoAimConfig.ENCODER_REVERSED ? -1.0 : 1.0) * motorDirSign;
    }

    /** 方向、PWM 范围每圈检查，Panels 里改了立刻生效；PWM 范围只在变化时才写硬件。 */
    private void applyServoConfig() {
        servos[0].setDirection(AutoAimConfig.SERVO_REVERSED_0
                ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servos[1].setDirection(AutoAimConfig.SERVO_REVERSED_1
                ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        if (AutoAimConfig.PWM_MIN_US != appliedPwmMin || AutoAimConfig.PWM_MAX_US != appliedPwmMax) {
            PwmControl.PwmRange range =
                    new PwmControl.PwmRange(AutoAimConfig.PWM_MIN_US, AutoAimConfig.PWM_MAX_US);
            for (ServoImplEx servo : servos) {
                servo.setPwmRange(range);
            }
            appliedPwmMin = AutoAimConfig.PWM_MIN_US;
            appliedPwmMax = AutoAimConfig.PWM_MAX_US;
        }
    }

    private void setPwmEnabled(boolean on) {
        if (!on) {
            servoPosition = Double.NaN;
        }
        if (on == pwmEnabled) {
            return;
        }
        for (ServoImplEx servo : servos) {
            if (on) {
                servo.setPwmEnable();
            } else {
                servo.setPwmDisable();
            }
        }
        pwmEnabled = on;
    }

    /** 云台角（rad）→ 舵机位置。舵机 1 额外加 SERVO_1_TRIM，让两个舵机不互相顶。 */
    private void setServoAngle(double angleRad) {
        servoPosition = clip(angleToServoPosition(angleRad), 0.0, 1.0);
        servos[0].setPosition(servoPosition);
        servos[1].setPosition(clip(servoPosition + AutoAimConfig.SERVO_1_TRIM, 0.0, 1.0));
        setPwmEnabled(true);
    }

    private static double angleToServoPosition(double angleRad) {
        return AutoAimConfig.CENTER_POS + Math.toDegrees(angleRad) / AutoAimConfig.TURRET_DEG_PER_POS;
    }

    private static double servoPositionToAngle(double position) {
        return Math.toRadians((position - AutoAimConfig.CENTER_POS) * AutoAimConfig.TURRET_DEG_PER_POS);
    }

    // ---------------------------------------------------------------
    // 限位 / 遥测
    // ---------------------------------------------------------------

    /** 可用范围 = 机械限位（留 LIMIT_MARGIN）∩ 舵机 0~1 能到的范围。 */
    private static double minAngle() {
        double mech = Math.toRadians(AutoAimConfig.MIN_ANGLE_DEG + AutoAimConfig.LIMIT_MARGIN_DEG);
        double servo = Math.min(servoPositionToAngle(0.0), servoPositionToAngle(1.0));
        return Math.max(mech, servo);
    }

    private static double maxAngle() {
        double mech = Math.toRadians(AutoAimConfig.MAX_ANGLE_DEG - AutoAimConfig.LIMIT_MARGIN_DEG);
        double servo = Math.max(servoPositionToAngle(0.0), servoPositionToAngle(1.0));
        return Math.min(mech, servo);
    }

    private void sendTelemetry() {
        String state;
        if (holding) {
            state = String.format("HOLD %.0f°", Math.toDegrees(holdAngle));
        } else if (!enabled) {
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
        telemetry.addData("Turret Cmd", "%.1f° (trim %.2f°)",
                Math.toDegrees(commandAngle), Math.toDegrees(trim));
        telemetry.addData("Turret Servo Pos", "%.3f", servoPosition);
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
