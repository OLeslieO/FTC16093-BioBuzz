package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * 双舵机 intake（三档：收回 / 中间 / 伸出）+ flower 舵机（三档：top / middle / bottom）。
 *
 * <p>两个 intake 舵机镜像安装，左右各自一组档位值（分别标定）。
 * 档位值是 {@code public static}，可以在 Panels 里实时调，下一个循环生效。
 *
 * <p>flower 和 intake 联动：
 * <ul>
 *   <li>放 intake（EXTENDED / MIDDLE）：flower 先到 MIDDLE，等 {@link #DEPLOY_DELAY_MS} 后 intake 再到目标档位</li>
 *   <li>做 flower：flower 到 BOTTOM（intake 必须已经在 EXTENDED / MIDDLE，否则忽略）</li>
 *   <li>收 flower：flower 回 MIDDLE</li>
 *   <li>收 intake：intake 到 RETRACTED，等 {@link #RETRACT_DELAY_MS} 后 flower 到 TOP</li>
 * </ul>
 * 延时是非阻塞的，在 {@link #periodic()} 里推进。
 *
 * <p>{@link #setFlowerEnabled(boolean) setFlowerEnabled(false)} 后：不再写 flower 舵机，
 * flower 相关方法都不生效，intake 换档直接切过去（没有联动和延时）。
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

    public enum FlowerState {
        TOP,
        MIDDLE,
        BOTTOM
    }

    public static String LEFT_SERVO_NAME = "leftIntakeServo";
    public static String RIGHT_SERVO_NAME = "rightIntakeServo";
    public static String FLOWER_SERVO_NAME = "flower";

    // 左舵机档位
    public static double RETRACTED_POSITION = 0.13;
    public static double MIDDLE_POSITION = 0.36;
    public static double EXTENDED_POSITION = 0.38;

    // 右舵机档位
    public static double RIGHT_RETRACTED_POSITION = 0.13;
    public static double RIGHT_MIDDLE_POSITION = 0.29;
    public static double RIGHT_EXTENDED_POSITION = 0.42;

    // flower 档位
    public static double FLOWER_TOP_POSITION = 0;
    public static double FLOWER_MIDDLE_POSITION = 0.35;
    public static double FLOWER_BOTTOM_POSITION = 0.96;

    /** 放 intake 时：flower 到 MIDDLE 后等多久再动 intake。 */
    public static double DEPLOY_DELAY_MS = 500;
    /** 收 intake 时：intake 到 RETRACTED 后等多久再把 flower 收到 TOP。 */
    public static double RETRACT_DELAY_MS = 500;

    private final Servo leftServo;
    private final Servo rightServo;
    private final Servo flowerServo;

    /** 当前写给 intake 舵机的档位。 */
    private State state = State.RETRACTED;
    /** 当前写给 flower 舵机的档位。 */
    private FlowerState flowerState = FlowerState.TOP;

    /** 等延时结束后要切到的 intake 档位；null 表示没有。 */
    private State pendingState = null;
    /** 等延时结束后要切到的 flower 档位；null 表示没有。 */
    private FlowerState pendingFlowerState = null;
    private final ElapsedTime pendingTimer = new ElapsedTime();

    /** false = 禁用 flower，并忽略 flower / intake 联动。 */
    private boolean flowerEnabled = true;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);
        flowerServo = hardwareMap.get(Servo.class, FLOWER_SERVO_NAME);
    }

    /**
     * 禁用 / 启用 flower。禁用时不再写 flower 舵机，intake 换档直接切过去（忽略联动）。
     */
    public void setFlowerEnabled(boolean enabled) {
        flowerEnabled = enabled;
        if (!enabled) {
            // 正在等的联动全部取消，intake 直接到最终档位
            if (pendingState != null) {
                state = pendingState;
            }
            pendingState = null;
            pendingFlowerState = null;
        }
    }

    public boolean isFlowerEnabled() {
        return flowerEnabled;
    }

    /** 切 intake 档位，flower 按联动规则跟着走（flower 禁用时直接切）。 */
    public void setState(State target) {
        if (!flowerEnabled) {
            pendingState = null;
            pendingFlowerState = null;
            state = target;
        } else if (target == State.RETRACTED) {
            retract();
        } else {
            deploy(target);
        }
    }

    /** 当前写给 intake 舵机的档位（延时中还没切过去的不算）。 */
    public State getState() {
        return state;
    }

    /** intake 最终要到的档位（包括还在等延时的）。 */
    public State getTargetState() {
        return pendingState != null ? pendingState : state;
    }

    public FlowerState getFlowerState() {
        return flowerState;
    }

    public void retract() {
        if (!flowerEnabled) {
            setState(State.RETRACTED);
            return;
        }
        pendingState = null;
        boolean intakeMoved = state != State.RETRACTED;
        state = State.RETRACTED;
        if (intakeMoved && RETRACT_DELAY_MS > 0) {
            pendingFlowerState = FlowerState.TOP;
            pendingTimer.reset();
        } else {
            pendingFlowerState = null;
            flowerState = FlowerState.TOP;
        }
    }

    public void middle() {
        setState(State.MIDDLE);
    }

    public void extend() {
        setState(State.EXTENDED);
    }

    /** flower 先到 MIDDLE，必要时等 DEPLOY_DELAY_MS 再把 intake 放到 target。 */
    private void deploy(State target) {
        pendingFlowerState = null;
        boolean flowerMoved = flowerState != FlowerState.MIDDLE;
        flowerState = FlowerState.MIDDLE;
        if (flowerMoved && DEPLOY_DELAY_MS > 0) {
            pendingState = target;
            pendingTimer.reset();
        } else {
            pendingState = null;
            state = target;
        }
    }

    /** intake 已经放到 EXTENDED / MIDDLE（且不在延时中）。 */
    public boolean isDeployed() {
        return state != State.RETRACTED && pendingState == null && pendingFlowerState == null;
    }

    /**
     * 做 flower：flower 到 BOTTOM。intake 没放下去时忽略。
     *
     * @return 是否生效
     */
    public boolean flowerDown() {
        if (!flowerEnabled || !isDeployed()) {
            return false;
        }
        flowerState = FlowerState.BOTTOM;
        return true;
    }

    /** 收 flower：flower 回 MIDDLE。intake 收着时 flower 应该在 TOP，所以忽略。 */
    public void flowerUp() {
        if (!flowerEnabled || !isDeployed()) {
            return;
        }
        flowerState = FlowerState.MIDDLE;
    }

    /** BOTTOM ↔ MIDDLE 切换。 */
    public void toggleFlower() {
        if (flowerState == FlowerState.BOTTOM) {
            flowerUp();
        } else {
            flowerDown();
        }
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

    /** 当前档位对应的右舵机位置。 */
    public double getRightTargetPosition() {
        switch (state) {
            case EXTENDED:
                return RIGHT_EXTENDED_POSITION;
            case MIDDLE:
                return RIGHT_MIDDLE_POSITION;
            case RETRACTED:
            default:
                return RIGHT_RETRACTED_POSITION;
        }
    }

    /** 当前档位对应的 flower 舵机位置。 */
    public double getFlowerTargetPosition() {
        switch (flowerState) {
            case BOTTOM:
                return FLOWER_BOTTOM_POSITION;
            case MIDDLE:
                return FLOWER_MIDDLE_POSITION;
            case TOP:
            default:
                return FLOWER_TOP_POSITION;
        }
    }

    /** 每个循环推进延时并把当前档位写给舵机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        if (pendingState != null && pendingTimer.milliseconds() >= DEPLOY_DELAY_MS) {
            state = pendingState;
            pendingState = null;
        }
        if (pendingFlowerState != null && pendingTimer.milliseconds() >= RETRACT_DELAY_MS) {
            flowerState = pendingFlowerState;
            pendingFlowerState = null;
        }

        leftServo.setPosition(Range.clip(getRightTargetPosition(), 0.0, 1.0));
        rightServo.setPosition(Range.clip(getTargetPosition(), 0.0, 1.0));
        if (flowerEnabled) {
            flowerServo.setPosition(Range.clip(getFlowerTargetPosition(), 0.0, 1.0));
        }
    }
}
