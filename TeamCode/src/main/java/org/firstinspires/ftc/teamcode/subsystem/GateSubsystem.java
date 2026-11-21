package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;


@Configurable
public class GateSubsystem extends SubsystemBase {

    public enum State {
        SHOOT,
        STOP
    }

    public static String GATE_NAME = "gate";

    public static double SHOOT_POSITION = 0.15;
    public static double STOP_POSITION = 0;

    private final Servo gate;

    private State state = State.STOP;

    public GateSubsystem(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, GATE_NAME);
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

    public double getTargetPosition() {
        switch (state) {
            case SHOOT:
                return SHOOT_POSITION;
            case STOP:
            default:
                return STOP_POSITION;
        }
    }

    @Override
    public void periodic() {
        gate.setPosition(Range.clip(getTargetPosition(), 0.0, 1.0));
    }
}
