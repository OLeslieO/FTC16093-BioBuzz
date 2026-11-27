package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class TransferSubsystem extends SubsystemBase {

    public enum State {
        INTAKE,
        OUTAKE,
        SHOOT,
        STOP,
        CUSTOM
    }

    public static String INTAKE_MOTOR_NAME = "intake";
    public static String PRE_SHOOTER_MOTOR_NAME = "preShooter";

    // ---------- 各档功率：intake 和 preShooter 分开给 ----------
    public static double INTAKE_INTAKE_POW = 1.0;
    public static double INTAKE_PRE_SHOOTER_POW = 1.0;

    public static double OUTAKE_INTAKE_POW = -0.9;
    public static double OUTAKE_PRE_SHOOTER_POW = 0;

    public static double SHOOT_INTAKE_POW = 0.85;
    public static double SHOOT_PRE_SHOOTER_POW = 0.95;

    private final DcMotorEx intake;
    private final DcMotorEx preShooter;

    private State state = State.STOP;
    private double customIntakePower = 0.0;
    private double customPreShooterPower = 0.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter = hardwareMap.get(DcMotorEx.class, PRE_SHOOTER_MOTOR_NAME);
        preShooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void accelerate() {
        setState(State.INTAKE);
    }

    public void idle() {
        setState(State.OUTAKE);
    }

    public void stop() {
        setState(State.STOP);
    }

    /** 两个电机给同一个功率。 */
    public void setPower(double power) {
        setPower(power, power);
    }

    /** intake 和 preShooter 分别给功率。 */
    public void setPower(double intakePower, double preShooterPower) {
        customIntakePower = intakePower;
        customPreShooterPower = preShooterPower;
        setState(State.CUSTOM);
    }

    public double getIntakePower() {
        switch (state) {
            case INTAKE:
                return INTAKE_INTAKE_POW;
            case OUTAKE:
                return OUTAKE_INTAKE_POW;
            case SHOOT:
                return SHOOT_INTAKE_POW;
            case CUSTOM:
                return customIntakePower;
            case STOP:
            default:
                return 0.0;
        }
    }

    public double getPreShooterPower() {
        switch (state) {
            case INTAKE:
                return INTAKE_PRE_SHOOTER_POW;
            case OUTAKE:
                return OUTAKE_PRE_SHOOTER_POW;
            case SHOOT:
                return SHOOT_PRE_SHOOTER_POW;
            case CUSTOM:
                return customPreShooterPower;
            case STOP:
            default:
                return 0.0;
        }
    }

    @Override
    public void periodic() {
        intake.setPower(getIntakePower());
        preShooter.setPower(getPreShooterPower());
    }
}
