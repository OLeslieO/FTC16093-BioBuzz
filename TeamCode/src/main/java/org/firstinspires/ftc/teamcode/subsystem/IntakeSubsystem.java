package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * 双舵机 intake，三档状态机：收回 / 中间 / 伸出。
 *
 * <p>两个舵机镜像安装：左舵机位置 = 档位值，右舵机位置 = 1 - 档位值。
 * 档位值是 {@code public static}，可以在 Panels 里实时调，下一个循环生效。
 */
@Configurable
public class IntakeSubsystem extends SubsystemBase {

    public enum State {
        /** 完全收回。 */
        RETRACTED,
        /** 中间位置。 */
        MIDDLE,
        /** 完全伸出。 */
        EXTENDED
    }

    public static String LEFT_SERVO_NAME = "leftIntakeServo";
    public static String RIGHT_SERVO_NAME = "rightIntakeServo";

    public static double RETRACTED_POSITION = 0.0;
    public static double MIDDLE_POSITION = 0.1;
    public static double EXTENDED_POSITION = 0.45;

    private final Servo leftServo;
    private final Servo rightServo;

    private State state = State.RETRACTED;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void retract() {
        setState(State.RETRACTED);
    }

    public void middle() {
        setState(State.MIDDLE);
    }

    public void extend() {
        setState(State.EXTENDED);
    }

    /** 当前档位对应的左舵机位置。 */
    public double getTargetPosition() {
        switch (state) {
            case EXTENDED:
                return EXTENDED_POSITION;
            case MIDDLE:
                return MIDDLE_POSITION;
            case RETRACTED:
            default:
                return RETRACTED_POSITION;
        }
    }

    /** 每个循环把当前档位写给舵机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        double position = getTargetPosition();
        leftServo.setPosition(Range.clip(position, 0.0, 1.0));
        rightServo.setPosition(Range.clip(1.0 - position, 0.0, 1.0));
    }
}
