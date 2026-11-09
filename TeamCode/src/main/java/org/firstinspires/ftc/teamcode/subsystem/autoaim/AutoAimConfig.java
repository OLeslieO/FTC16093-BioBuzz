package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.bylazar.configurables.annotations.Configurable;

/**
 * 云台自动瞄准的全部可调参数，都能在 Panels 里实时修改。
 *
 * <p>所有数值目前都是占位默认值，必须按实机测量 / 整定后填写，
 * 清单见 {@link AutoAimSubsystem} 的类注释。
 *
 * <p>角度约定与 Pedro 一致：逆时针为正，0 度 = 机器人正前方。
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
    public static double TICKS_PER_TURRET_REV = 1000.0;

    /** 编码器读数为 0 时，云台相对机器人正前方的角度（度）。开机时手动摆正前方则为 0。 */
    public static double ZERO_OFFSET_DEG = 0.0;

    // ---------------------------------------------------------------
    // 机械范围
    // ---------------------------------------------------------------

    /** 云台允许转到的最小 / 最大角度（度，相对机器人正前方），由线缆和机械限位决定。 */
    public static double MIN_ANGLE_DEG = -180.0;
    public static double MAX_ANGLE_DEG = 180.0;

    // ---------------------------------------------------------------
    // 几何
    // ---------------------------------------------------------------

    /** 云台旋转轴相对机器人中心（Pedro 定位点）的偏移，英寸。前为正。 */
    public static double PIVOT_OFFSET_FORWARD_IN = 0.0;

    /** 云台旋转轴相对机器人中心的偏移，英寸。左为正。 */
    public static double PIVOT_OFFSET_LEFT_IN = 0.0;

    // ---------------------------------------------------------------
    // 控制
    // ---------------------------------------------------------------

    /** 比例项：每弧度误差对应的功率。 */
    public static double kP = 1.2;

    /** 微分项：作用在误差变化率上（每 rad/s 对应的功率）。 */
    public static double kD = 0.05;

    /** 底盘旋转前馈：底盘每转 1 rad/s，云台反向补偿的功率。 */
    public static double kV_OMEGA = 0.08;

    /** 克服静摩擦的功率，误差大于死区时按误差方向叠加。 */
    public static double kS = 0.03;

    public static double MAX_POWER = 0.8;

    /** 误差小于这个角度（度）就不再输出，避免抖动。 */
    public static double DEADBAND_DEG = 0.3;

    /** 误差小于这个角度（度）视为"已瞄准"。 */
    public static double LOCK_TOLERANCE_DEG = 2.0;

    /** 误差变化率 / 底盘角速度的低通滤波系数（0~1，越大越跟手）。 */
    public static double FILTER_ALPHA = 0.5;

    /** 离机械限位这么近（度）时，禁止继续朝限位方向输出功率。 */
    public static double LIMIT_MARGIN_DEG = 2.0;

    private AutoAimConfig() {
    }
}
