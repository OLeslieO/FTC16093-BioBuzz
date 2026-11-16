package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;


@Configurable
public class PollenGateSubsystem extends SubsystemBase {

    public enum State {
        SHOOT,
        STOP
    }

    public static String POLLEN_GATE_NAME = "pollenGate";

    public static double POLLEN_SHOOT_POSITION = 0.5;
    public static double POLLEN_STOP_POSITION = 0.9;

    private final Servo pollenGate;

    private State state = State.STOP;

    public PollenGateSubsystem(HardwareMap hardwareMap) {
        pollenGate = hardwareMap.get(Servo.class, POLLEN_GATE_NAME);
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
                return POLLEN_SHOOT_POSITION;
            case STOP:
            default:
                return POLLEN_STOP_POSITION;
        }
    }

    @Override
    public void periodic() {
        pollenGate.setPosition(Range.clip(getTargetPosition(), 0.0, 1.0));
    }
}
