package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoAimConfig {


    public static String SERVO_NAME_0 = "lala1";
    public static String SERVO_NAME_1 = "lala2";

    /** 正功率 = 云台逆时针转（俯视），编码器读数随之增大"*/
    public static boolean SERVO_REVERSED_0 = false;
    public static boolean SERVO_REVERSED_1 = false;

    /** Through Bore 电机端口 */
    public static String ENCODER_NAME = "intake1";

    public static boolean ENCODER_REVERSED = false;

    public static double ENCODER_CPR = 8192.0;

    public static double ENCODER_TO_TURRET = 0.14814815;

    public static double ticksPerTurretRev() {
        return ENCODER_CPR / ENCODER_TO_TURRET;
    }

    public static double ZERO_OFFSET_DEG = 0.0;

    public static double MIN_ANGLE_DEG = -100.0;
    public static double MAX_ANGLE_DEG = 100.0;

    public static double LIMIT_MARGIN_DEG = 3.0;

    public enum Turret {
        NECTAR,
        POLLEN
    }

    public static Turret ACTIVE_TURRET = Turret.NECTAR;

    public static double NECTAR_PIVOT_OFFSET_FORWARD_IN = -1.033;

    public static double NECTAR_PIVOT_OFFSET_LEFT_IN = -2.44;

    public static double POLLEN_PIVOT_OFFSET_FORWARD_IN = 2.75591;

    public static double POLLEN_PIVOT_OFFSET_LEFT_IN = 4.72441;

    public static double pivotOffsetForwardIn() {
        return ACTIVE_TURRET == Turret.POLLEN ? POLLEN_PIVOT_OFFSET_FORWARD_IN : NECTAR_PIVOT_OFFSET_FORWARD_IN;
    }

    public static double pivotOffsetLeftIn() {
        return ACTIVE_TURRET == Turret.POLLEN ? POLLEN_PIVOT_OFFSET_LEFT_IN : NECTAR_PIVOT_OFFSET_LEFT_IN;
    }

    public static void toggleTurret() {
        ACTIVE_TURRET = ACTIVE_TURRET == Turret.NECTAR ? Turret.POLLEN : Turret.NECTAR;
    }

    public static double YAW_TRIM_DEG = 0.0;


    public static double KP_POS = 3.0;

    public static double MAX_TURRET_VEL = 2.269;

    public static double DEADBAND_DEG = 0.6;


    public static double kS = 0.14;

    public static double kV = 0.11;

    public static double kA = 0.0;

    public static double KS_RAMP_VEL = 0.15;


    public static double FF_DEADBAND = 0.05;

    public static double KP_VEL = 0.06;

    public static double VEL_FILTER_ALPHA = 0.5;

    public static double CMD_ACCEL_FILTER_ALPHA = 0.15;

    public static double MAX_POWER = 1.0;


    public static double CHASSIS_VEL_FILTER_ALPHA = 0.3;

    public static double CHASSIS_ACCEL_FILTER_ALPHA = 0.4;

    public static double OMEGA_FILTER_ALPHA = 0.1;

    public static double POSE_LATENCY = 0.015;


    public static double FORWARD_BRAKE_DECEL = 30.0;

    public static double LATERAL_BRAKE_DECEL = 50.0;

    public static double BRAKE_PREDICTION_WEIGHT = 0.55;


    public static double SHOOT_DELAY = 0.08;

    public static double VELOCITY_INHERITANCE = 1.0;

    public static int LEAD_ITERATIONS = 3;

    public static double[] FLIGHT_DIST_IN = {24.0, 48.0, 72.0, 96.0, 120.0, 144.0};
    public static double[] FLIGHT_TIME_SEC = {0.45, 0.55, 0.65, 0.75, 0.85, 0.95};

    public static double MAX_SHOOT_SPEED = 60.0;

    public static double MAX_STATIONARY_SHOOT_SPEED = 5.0;


    public static double AIM_HALF_WIDTH_IN = 4.0;

    public static double MIN_LOCK_TOLERANCE_DEG = 1.0;
    public static double MAX_LOCK_TOLERANCE_DEG = 10.0;
    public static double LOCK_MAX_RATE_ERROR_DEG = 30.0;

    private AutoAimConfig() {
    }
}
