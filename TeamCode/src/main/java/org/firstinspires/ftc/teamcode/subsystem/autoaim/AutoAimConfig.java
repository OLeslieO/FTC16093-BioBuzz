package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoAimConfig {


    public static String SERVO_NAME_0 = "lalaR";
    public static String SERVO_NAME_1 = "lalaL";

    /** 舵机位置增大 = 云台逆时针转（俯视），编码器读数随之增大 */
    public static boolean SERVO_REVERSED_0 = false;
    public static boolean SERVO_REVERSED_1 = false;

    /** Through Bore 电机端口 */
    public static String ENCODER_NAME = "preShooter";

    public static boolean ENCODER_REVERSED = false;

    public static double ENCODER_CPR = 8192.0;

    public static double ENCODER_TO_TURRET = 0.569;
    // servo 3/ turret 4

    public static double ticksPerTurretRev() {
        return ENCODER_CPR / ENCODER_TO_TURRET;
    }

    public static double ZERO_OFFSET_DEG = 0.0;

    public static double MIN_ANGLE_DEG = -170.0;
    public static double MAX_ANGLE_DEG = 170.0;

    public static double LIMIT_MARGIN_DEG = 3.0;

    /** 云台旋转轴相对底盘旋转中心的偏移（英寸）。旋转轴就在底盘旋转中心上方，所以是 0。 */
    public static double PIVOT_OFFSET_FORWARD_IN = 0.0;
    public static double PIVOT_OFFSET_LEFT_IN = 0.0;

    public static double YAW_TRIM_DEG = 0.0;


    // ---- 舵机位置模式 ----
    // 舵机位置 = CENTER_POS + 云台角度(°) / TURRET_DEG_PER_POS。
    // 用 TurretPositionServoTest MODE 4 标定，标完把结果抄到这里。

    /** 舵机位置变化 1.0 云台转多少度。默认按舵机 300° 行程、舵机:云台 = 4:3 估的，需实测。 */
    public static double TURRET_DEG_PER_POS = 403;
    /** 云台 0°（正前方）对应的舵机位置。 */
    public static double CENTER_POS = 0.5;

    /** 舵机 PWM 范围（µs）。Axon / goBILDA 全行程一般 500~2500，SDK 默认 600~2400。 */
    public static double PWM_MIN_US = 500.0;
    public static double PWM_MAX_US = 2500.0;

    /** 舵机 1 相对舵机 0 的位置微调，两个舵机咬同一个齿轮，0 点不一致会互相顶着发热。 */
    public static double SERVO_1_TRIM = 0.0;

    /** 超前补偿（s）：目标在动时多转 前馈角速度 × 这个时间，抵消舵机跟随滞后。追不上就加大。 */
    public static double SERVO_LEAD_TIME = 0.05;

    /** 指令角度限速（rad/s），0 = 不限速。舵机冲太猛、过冲大时打开。 */
    public static double MAX_TURRET_VEL = 0.0;

    /** 用编码器修正稳态误差的积分增益（1/s），0 = 纯开环。齿隙或标定不准时再开。 */
    public static double TRIM_KI = 0.0;
    /** 误差小于这个才积分。 */
    public static double TRIM_WINDOW_DEG = 3.0;
    public static double TRIM_MAX_DEG = 5.0;

    /** 积分修正的死区。 */
    public static double DEADBAND_DEG = 0.5;

    /** 前馈角速度（rad/s）的软死区，底盘静止时滤掉定位噪声。 */
    public static double FF_DEADBAND = 0.05;

    public static double VEL_FILTER_ALPHA = 0.5;


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
