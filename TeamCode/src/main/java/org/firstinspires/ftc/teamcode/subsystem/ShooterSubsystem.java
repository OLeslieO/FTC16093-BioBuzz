package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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
    public static String POLLEN_SHOOTER_NAME = "PollenShooter";

    public static double ACCELERATE_VEL = 1640;
    public static double IDLE_VEL = 600.0;
    public static double STOP_VEL = 0.0;

    public static double setP = 34;
    public static double setI = 0;
    public static double setD = 0;
    public static double setF = 16.5;

    private final DcMotorEx PollenShooter;
    private final DcMotorEx NectarShooter;

    private State state = State.IDLE;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        PollenShooter = hardwareMap.get(DcMotorEx.class, POLLEN_SHOOTER_NAME);
        NectarShooter = hardwareMap.get(DcMotorEx.class, NECTAR_SHOOTER_NAME);
        PollenShooter.setVelocityPIDFCoefficients(setP, setI, setD, setF);

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
                return IDLE_VEL;
        }
    }

    /** 每个循环把当前速度写给电机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        double velocity = getTargetVelocity();
        PollenShooter.setVelocity(velocity);
        NectarShooter.setVelocity(velocity);
    }
}
