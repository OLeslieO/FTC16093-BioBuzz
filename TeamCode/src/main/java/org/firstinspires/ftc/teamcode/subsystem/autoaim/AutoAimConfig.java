package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.bylazar.configurables.annotations.Configurable;

/**
 * 云台自动瞄准的全部可调参数，都能在 Panels 里实时修改。
 *
 * <p>所有数值目前都是占位默认值，必须按实机测量 / 整定后填写，
 * 清单见 {@link AutoAimSubsystem} 的类注释。
 *
 * <p>角度约定与 Pedro 一致：逆时针为正，0 度 = 机器人正前方。
 * 长度单位英寸，时间单位秒。
 */
@Configurable
public class AutoAimConfig {

    // ---------------------------------------------------------------
    // 硬件
    // ---------------------------------------------------------------

    /** 云台电机在配置文件里的名字。 */
    public static String MOTOR_NAME = "turret";

    /** 让"正功率 = 云台逆时针转（俯视），编码器随之增大"。方向反了就翻转这个。 */
    public static boolean MOTOR_REVERSED = false;

    /** 云台转一整圈（360°）对应的编码器 ticks = 电机每圈 ticks × 总减速比。 */
    // 6.75 *
    public static double TICKS_PER_TURRET_REV = 1000.0;

    /** 编码器读数为 0 时，云台相对机器人正前方的角度（度）。开机时手动摆正前方则为 0。 */
    public static double ZERO_OFFSET_DEG = 0.0;

    // ---------------------------------------------------------------
    // 机械范围
    // ---------------------------------------------------------------

    /** 云台允许转到的最小 / 最大角度（度，相对机器人正前方），由线缆和机械限位决定。 */
    public static double MIN_ANGLE_DEG = -120.0;
    public static double MAX_ANGLE_DEG = 120.0;

    /** 离机械限位这么近（度）时，禁止继续朝限位方向输出功率。 */
    public static double LIMIT_MARGIN_DEG = 2.0;

    // ---------------------------------------------------------------
    // 几何
    // ---------------------------------------------------------------

    /** 云台旋转轴相对机器人中心（Pedro 定位点）的偏移，英寸。前为正。 */
    public static double PIVOT_OFFSET_FORWARD_IN = 0.0;

    /** 云台旋转轴相对机器人中心的偏移，英寸。左为正。 */
    public static double PIVOT_OFFSET_LEFT_IN = 0.0;

    /** 固定的瞄准角修正（度），用来抵消机械 / 安装造成的系统性偏差。逆时针为正。 */
    public static double YAW_TRIM_DEG = 0.0;

    // ---------------------------------------------------------------
    // 位置环（输出目标角速度）
    // ---------------------------------------------------------------

    /** 每弧度误差对应的目标角速度（1/s）。 */
    public static double KP_POS = 8.0;

    /** 目标角速度上限（rad/s），应略低于云台实测最大转速。 */
    public static double MAX_TURRET_VEL = 10.0;

    /** 误差小于这个角度（度）时位置环不出力，只保留前馈，避免抖动。 */
    public static double DEADBAND_DEG = 0.3;

    // ---------------------------------------------------------------
    // 电机前馈 + 速度环（目标角速度 -> 功率）
    // ---------------------------------------------------------------

    /** 克服静摩擦的功率，按目标角速度方向叠加。 */
    public static double kS = 0.03;

    /** 每 rad/s 目标角速度对应的功率（在 {@link #NOMINAL_VOLTAGE} 下测得）。 */
    public static double kV = 0.08;

    /** 每 rad/s² 目标角加速度对应的功率。 */
    public static double kA = 0.005;

    /** 目标角速度低于这个值（rad/s）时不叠加 kS，防止静止时来回抖。 */
    public static double KS_MIN_VEL = 0.05;

    /** 速度环比例项：每 rad/s 速度误差对应的功率。先用 0，前馈调好后再加。 */
    public static double KP_VEL = 0.0;

    /** 目标角加速度的低通系数（0~1，越大越跟手、越吵）。 */
    public static double CMD_ACCEL_FILTER_ALPHA = 0.15;

    public static double MAX_POWER = 1.0;

    // ---------------------------------------------------------------
    // 电压补偿
    // ---------------------------------------------------------------

    /** 整定 kS / kV / kA 时的电池电压。功率会按 NOMINAL / 当前电压 放大。 */
    public static double NOMINAL_VOLTAGE = 12.5;

    /** 读电压的间隔（秒）。读电压比较慢，不必每帧读。 */
    public static double VOLTAGE_READ_PERIOD = 0.25;

    // ---------------------------------------------------------------
    // 底盘运动估计
    // ---------------------------------------------------------------

    /** 底盘速度（车体坐标系）的低通系数。 */
    public static double CHASSIS_VEL_FILTER_ALPHA = 0.3;

    /** 底盘加速度（由滤波后速度差分得到）的低通系数。 */
    public static double CHASSIS_ACCEL_FILTER_ALPHA = 0.4;

    /** 底盘角速度的低通系数。 */
    public static double OMEGA_FILTER_ALPHA = 0.7;

    /** 定位数据的延迟（秒）：位姿 / 速度都按这个时间往前外推。 */
    public static double POSE_LATENCY = 0.015;

    // ---------------------------------------------------------------
    // 刹车预测
    // ---------------------------------------------------------------

    /** 松开摇杆刹车时，车头方向的减速度（in/s²），实测。 */
    public static double FORWARD_BRAKE_DECEL = 30.0;

    /** 松开摇杆刹车时，横移方向的减速度（in/s²），实测。 */
    public static double LATERAL_BRAKE_DECEL = 50.0;

    /** 刹车时预测加速度 = 实测 × (1 - w) + 刹车减速度 × w。 */
    public static double BRAKE_PREDICTION_WEIGHT = 0.55;

    // ---------------------------------------------------------------
    // 边走边射
    // ---------------------------------------------------------------

    /** 从"决定开火"到球离开射球机的机械延迟（秒）。 */
    public static double SHOOT_DELAY = 0.08;

    /** 球继承底盘速度的比例。理论 1.0，球和轮打滑时会偏小，实测修正。 */
    public static double VELOCITY_INHERITANCE = 1.0;

    /** 求解提前量时的迭代次数。2~3 次就够收敛。 */
    public static int LEAD_ITERATIONS = 3;

    /**
     * 球的滞空时间表：距离（英寸，必须递增）-> 滞空时间（秒）。
     * 两个数组长度必须相同；超出范围时取两端的值。
     */
    public static double[] FLIGHT_DIST_IN = {24.0, 48.0, 72.0, 96.0, 120.0, 144.0};
    public static double[] FLIGHT_TIME_SEC = {0.45, 0.55, 0.65, 0.75, 0.85, 0.95};

    /** 边走边射开启时，底盘速度超过这个值（in/s）就不算"已瞄准"，不允许开火。 */
    public static double MAX_SHOOT_SPEED = 60.0;

    /** 边走边射关闭时（不算提前量），底盘速度超过这个值（in/s）就不算"已瞄准"。 */
    public static double MAX_STATIONARY_SHOOT_SPEED = 5.0;

    // ---------------------------------------------------------------
    // 锁定判定
    // ---------------------------------------------------------------

    /**
     * 目标有效半宽（英寸）。容差角 = atan(半宽 / 距离)，近处自然放宽、远处收紧。
     */
    public static double AIM_HALF_WIDTH_IN = 4.0;

    /** 容差角的上下限（度）。 */
    public static double MIN_LOCK_TOLERANCE_DEG = 1.0;
    public static double MAX_LOCK_TOLERANCE_DEG = 10.0;

    /** 云台实际角速度与应有角速度相差超过这个值（度/秒）时，不算锁定。 */
    public static double LOCK_MAX_RATE_ERROR_DEG = 30.0;

    private AutoAimConfig() {
    }
}
