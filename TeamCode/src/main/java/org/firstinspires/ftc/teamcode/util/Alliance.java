package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * 联盟（红 / 蓝）。
 *
 * <p>联盟信息保存在静态字段里，Robot Controller 进程不重启就会一直保留，
 * 所以自动阶段 {@link #set(Alliance)} 之后，手动阶段直接 {@link #get()} 即可，
 * 和 {@code PPDriveSubsystem} 传递 pose 的方式一致。
 *
 * <p>典型用法：
 * <pre>
 * // 自动阶段 initialize()
 * Alliance.set(Alliance.RED);
 *
 * // 手动阶段 initialize()
 * Alliance alliance = Alliance.get();          // 没设置时默认 RED
 * Pose aimPoint = alliance.getAimPose();        // 云台瞄准点
 * double aim = alliance.aimHeadingTo(drive.getPose());
 * </pre>
 */
public enum Alliance {
    RED,
    BLUE;

    /** 没有任何 OpMode 设置过联盟时的默认值。 */
    public static final Alliance DEFAULT = RED;

    private static Alliance current = null;

    // ---------------------------------------------------------------
    // 静态存取
    // ---------------------------------------------------------------

    /**
     * 当前联盟。没有设置过时返回 {@link #DEFAULT}。
     */
    public static Alliance get() {
        return current != null ? current : DEFAULT;
    }

    /**
     * 当前联盟是否被显式设置过（自动阶段或手柄选择）。
     */
    public static boolean isSet() {
        return current != null;
    }

    public static void set(Alliance alliance) {
        current = alliance;
    }

    /** 清除保存的联盟，之后 {@link #get()} 返回 {@link #DEFAULT}。 */
    public static void clear() {
        current = null;
    }

    public static boolean isRed() {
        return get() == RED;
    }

    public static boolean isBlue() {
        return get() == BLUE;
    }

    /**
     * 在 init 阶段用手柄选联盟：X（蓝）选 BLUE，B（红）选 RED。
     * 每个 init 循环调用一次。
     *
     * @return 这次调用是否改变了联盟
     */
    public static boolean selectWithGamepad(Gamepad gamepad) {
        Alliance selected = null;
        if (gamepad.x) {
            selected = BLUE;
        } else if (gamepad.b) {
            selected = RED;
        }
        if (selected == null || selected == current) {
            return false;
        }
        set(selected);
        return true;
    }

    // ---------------------------------------------------------------
    // 实例方法
    // ---------------------------------------------------------------

    /** 对方联盟。 */
    public Alliance other() {
        return this == RED ? BLUE : RED;
    }

    /** 本方云台瞄准点 X（Pedro 坐标系，英寸）。 */
    public double getAimX() {
        return this == RED ? FieldConstants.RED_AIM_X : FieldConstants.BLUE_AIM_X;
    }

    /** 本方云台瞄准点 Y（Pedro 坐标系，英寸）。 */
    public double getAimY() {
        return this == RED ? FieldConstants.RED_AIM_Y : FieldConstants.BLUE_AIM_Y;
    }

    /**
     * 本方云台瞄准点，供云台瞄准使用。heading 无意义，固定为 0。
     * 每次调用都重新读取 {@link FieldConstants}，这样 Panels 改值即时生效。
     */
    public Pose getAimPose() {
        return new Pose(getAimX(), getAimY(), 0.0);
    }

    /**
     * 从机器人当前位置指向本方瞄准点的场地绝对角度（弧度，范围 (-π, π]）。
     * 云台相对角 = 这个值 - 机器人 heading。
     */
    public double aimHeadingTo(Pose robotPose) {
        return aimHeadingTo(robotPose.x(), robotPose.y());
    }

    public double aimHeadingTo(double robotX, double robotY) {
        return Math.atan2(getAimY() - robotY, getAimX() - robotX);
    }

    /** 机器人到本方瞄准点的距离（英寸）。 */
    public double distanceToAim(Pose robotPose) {
        return Math.hypot(getAimX() - robotPose.x(), getAimY() - robotPose.y());
    }

    // ---------------------------------------------------------------
    // 镜像工具：路径按蓝方写一遍，红方自动翻转
    // ---------------------------------------------------------------

    /**
     * 把一个"按 {@link #BLUE} 写的" pose 转换到本联盟。
     * 蓝方原样返回；红方关于 X = 72 镜像（x -> 144 - x，heading -> π - heading）。
     */
    public Pose fromBlue(Pose bluePose) {
        return this == BLUE ? bluePose : mirror(bluePose);
    }

    /**
     * 把一个"按 {@link #RED} 写的" pose 转换到本联盟。
     */
    public Pose fromRed(Pose redPose) {
        return this == RED ? redPose : mirror(redPose);
    }

    /** 关于场地中线 X = 72 镜像一个 pose。 */
    public static Pose mirror(Pose pose) {
        return new Pose(
                mirrorX(pose.x()),
                pose.y(),
                mirrorHeading(pose.heading())
        );
    }

    public static double mirrorX(double x) {
        return 2.0 * FieldConstants.MIRROR_X - x;
    }

    /** 关于 Y 轴镜像一个角度（弧度）：θ -> π - θ，结果归一化到 (-π, π]。 */
    public static double mirrorHeading(double headingRadians) {
        double h = Math.PI - headingRadians;
        while (h > Math.PI) {
            h -= 2.0 * Math.PI;
        }
        while (h <= -Math.PI) {
            h += 2.0 * Math.PI;
        }
        return h;
    }
}
