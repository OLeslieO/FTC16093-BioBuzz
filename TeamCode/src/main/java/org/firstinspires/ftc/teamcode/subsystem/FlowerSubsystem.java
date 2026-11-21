package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;


@Configurable
public class FlowerSubsystem extends SubsystemBase {

    public enum State {
        RETRACTED,
        EXTENDED
    }

    public static String FLOWER_SERVO_NAME = "flower";

    public static double RETRACTED_POSITION = 0.35;
    public static double EXTENDED_POSITION = 0.96;

    private final Servo flowerServo;

    private State state = State.RETRACTED;

    public FlowerSubsystem(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.get(Servo.class, FLOWER_SERVO_NAME);
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

    public void extend() {
        setState(State.EXTENDED);
    }

    public void toggle() {
        setState(state == State.EXTENDED ? State.RETRACTED : State.EXTENDED);
    }

    public boolean isExtended() {
        return state == State.EXTENDED;
    }

    public double getTargetPosition() {
        return state == State.EXTENDED ? EXTENDED_POSITION : RETRACTED_POSITION;
    }

    @Override
    public void periodic() {
        flowerServo.setPosition(Range.clip(getTargetPosition(), 0.0, 1.0));
    }
}
