package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class TransferSubsystem extends SubsystemBase {

    public enum State {
        /** 加速。 */
        INTAKE,
        /** 空转。 */
        OUTAKE,
        /** 停止。 */
        STOP
    }

    public static String INTAKE_MOTOR_NAME = "intake";

    public static double INTAKE_POW = 1;
    public static double OUTAKE_POW = -0.8;
    public static double STOP_POW = 0.0;

    private final DcMotorEx intkae;

    private State state = State.STOP;

    public TransferSubsystem(HardwareMap hardwareMap) {
        intkae = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
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

    /** 当前档位对应的shooter速度 */
    public double getTargetPower() {
        switch (state) {
            case INTAKE:
                return INTAKE_POW;
            case OUTAKE:
                return OUTAKE_POW;
            case STOP:
                return STOP_POW;
            default:
                return STOP_POW;
        }
    }

    /** 每个循环把当前速度写给电机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        double power = getTargetPower();
        intkae.setPower(power);
    }
}
