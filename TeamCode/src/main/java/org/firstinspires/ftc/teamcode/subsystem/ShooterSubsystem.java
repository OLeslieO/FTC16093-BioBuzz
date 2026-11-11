package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Nectar shooter 飞轮，自己算功率的 前馈 + PID 速度控制（参数来自 ShooterFFPIDTest 实测）。
 *
 * power = 电压补偿 × (kS + kV·target) + kP·error + I + kD·(-dVel/dt)
 * 掉速超过 BOOST_THRESHOLD 时直接满功率。
 *
 * <p>所有参数都是 {@code public static}，可以在 Panels 里实时调，下一个循环生效。
 */
@Configurable
public class ShooterSubsystem extends SubsystemBase {

    public enum State {
        /** 加速。 */
        ACCELERATE,
        /** 空转。 */
        IDLE,
        /** 停止。 */
        STOP
    }

    public static String NECTAR_SHOOTER_NAME = "NectarShooter";

    // ---------- 目标（ticks/s） ----------
    public static double ACCELERATE_VEL = 1820;
    public static double IDLE_VEL = 600.0;
    public static double STOP_VEL = 0.0;

    // ---------- 前馈（按 12V 标定） ----------
    public static double kS = 0.08;
    public static double kV = 0.0005;
    public static double NOMINAL_VOLTAGE = 12.0;

    // ---------- 反馈 ----------
    public static double kP = 0.018;
    public static double kI = 0.0005;
    public static double kD = 0.0;
    /** 误差小于这个才积分（ticks/s）。 */
    public static double I_ZONE = 150;
    /** 积分项最大功率。 */
    public static double I_MAX = 0.15;

    // ---------- boost：掉速很多时满功率 ----------
    public static boolean USE_BOOST = true;
    public static double BOOST_THRESHOLD = 200;

    // ---------- 测量 ----------
    /** 速度低通系数，1 = 不滤波。 */
    public static double VELOCITY_FILTER_ALPHA = 0.6;
    /** 误差在这范围内算"到速"。 */
    public static double READY_TOLERANCE = 40;
    /** 电压读取间隔（秒），读电压比较慢，没必要每个循环读。 */
    public static double VOLTAGE_READ_INTERVAL = 0.1;

    private final DcMotorEx nectarShooter;
    private final VoltageSensor battery;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private State state = State.ACCELERATE;

    private double filteredVoltage;
    private double filteredVel = 0;
    private double integral = 0; // 已经乘过 kI，单位是功率
    private double lastTarget = Double.NaN;
    private boolean initialized = false;

    // 仅用于 telemetry
    private double error, ff, pTerm, dTerm, power;
    private boolean boosting;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        nectarShooter = hardwareMap.get(DcMotorEx.class, NECTAR_SHOOTER_NAME);
        nectarShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        nectarShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 编码器照样能读速度，但功率由我们自己算
        nectarShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().next();
        filteredVoltage = battery.getVoltage();
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void accelerate() {
        setState(State.ACCELERATE);
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void stop() {
        setState(State.STOP);
    }

    /** 当前档位对应的shooter速度 */
    public double getTargetVelocity() {
        switch (state) {
            case ACCELERATE:
                return ACCELERATE_VEL;
            case IDLE:
                return IDLE_VEL;
            case STOP:
                return STOP_VEL;
            default:
                return ACCELERATE_VEL;
        }
    }

    public double getVelocity() {
        return filteredVel;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public boolean isBoosting() {
        return boosting;
    }

    /** 到速了没有，可以用来决定什么时候开门。 */
    public boolean isReady() {
        return getTargetVelocity() > 0 && Math.abs(error) < READY_TOLERANCE;
    }

    /** 每个循环算一次功率写给电机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();

        if (voltageTimer.seconds() > VOLTAGE_READ_INTERVAL) {
            voltageTimer.reset();
            filteredVoltage += 0.3 * (battery.getVoltage() - filteredVoltage);
        }

        double measuredVel = nectarShooter.getVelocity();
        if (!initialized) {
            filteredVel = measuredVel;
            initialized = true;
        }
        double lastFilteredVel = filteredVel;
        filteredVel += Range.clip(VELOCITY_FILTER_ALPHA, 0.01, 1.0) * (measuredVel - filteredVel);

        double target = getTargetVelocity();
        if (target != lastTarget) {
            integral = 0;
            lastTarget = target;
        }

        if (target <= 0) {
            integral = 0;
            error = -filteredVel;
            ff = pTerm = dTerm = 0;
            boosting = false;
            power = 0; // 飞轮自己滑停，不反向刹车
        } else {
            error = target - filteredVel;
            double voltageComp = NOMINAL_VOLTAGE / Math.max(filteredVoltage, 6.0);

            ff = (kS + kV * target) * voltageComp;
            pTerm = kP * error;
            // D 作用在测量值上，切换目标不会尖峰
            dTerm = -kD * (filteredVel - lastFilteredVel) / dt;

            boosting = USE_BOOST && error > BOOST_THRESHOLD;

            if (boosting) {
                integral = 0;
                power = 1.0;
            } else {
                double unclipped = ff + pTerm + integral + dTerm;
                // 条件积分防饱和
                boolean saturatedHigh = unclipped >= 1.0 && error > 0;
                boolean saturatedLow = unclipped <= 0.0 && error < 0;
                if (Math.abs(error) < I_ZONE && !saturatedHigh && !saturatedLow) {
                    integral = Range.clip(integral + kI * error * dt, -I_MAX, I_MAX);
                }
                power = Range.clip(ff + pTerm + integral + dTerm, 0.0, 1.0);
            }
        }

        nectarShooter.setPower(power);
    }
}
