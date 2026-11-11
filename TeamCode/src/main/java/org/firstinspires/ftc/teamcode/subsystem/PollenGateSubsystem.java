package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * 发射口挡板舵机，两档：SHOOT（打开放球）/ STOP（关闭挡球）。
 *
 * <p>档位值是 {@code public static}，可以在 Panels 里实时调，下一个循环生效。
 */
@Configurable
public class PollenGateSubsystem extends SubsystemBase {

    public enum State {
        /** 打开，放球去发射。 */
        SHOOT,
        /** 关闭，挡住球。 */
        STOP
    }

    public static String GATE_SERVO_NAME = "pollenGate";

    public static double SHOOT_POSITION = 0.57;
    public static double STOP_POSITION = 0.37;

    private final Servo gate;

    private State state = State.STOP;

    public PollenGateSubsystem(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, GATE_SERVO_NAME);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void shoot() {
        setState(State.SHOOT);
    }

    public void stop() {
        setState(State.STOP);
    }

    /** 当前档位对应的舵机位置。 */
    public double getTargetPosition() {
        switch (state){
            case STOP:
                return SHOOT_POSITION;
            case SHOOT:
                return STOP_POSITION;
            default:
                return STOP_POSITION;
        }
    }

    /** 每个循环把当前档位写给舵机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        gate.setPosition(Range.clip(getTargetPosition(), 0.0, 1.0));
    }
}
