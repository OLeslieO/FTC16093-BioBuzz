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
    public static double INTAKE_POW = 1;
    public static double OUTAKE_POW = -0.9;
    public static double SHOOT_POW = 0.85;
    public static double STOP_POW = 0.0;

    private final DcMotorEx intake;
    private final DcMotorEx preShooter;

    private State state = State.STOP;
    private double customPower = 0.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter = hardwareMap.get(DcMotorEx.class,"preShooter");
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

    public void setPower(double power) {
        customPower = power;
        setState(State.CUSTOM);
    }

    public double getTargetPower() {
        switch (state) {
            case INTAKE:
                return INTAKE_POW;
            case OUTAKE:
                return OUTAKE_POW;
            case SHOOT:
                return SHOOT_POW;
            case STOP:
                return STOP_POW;
            case CUSTOM:
                return customPower;
            default:
                return STOP_POW;
        }
    }

    @Override
    public void periodic() {
        double power = getTargetPower();
        intake.setPower(power);
        preShooter.setPower(power);
    }
}
