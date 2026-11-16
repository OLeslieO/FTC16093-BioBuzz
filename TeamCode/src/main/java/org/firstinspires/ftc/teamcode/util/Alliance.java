package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;


public enum Alliance {
    RED,
    BLUE;

    public static final Alliance DEFAULT = RED;

    private static Alliance current = null;


    public static Alliance get() {
        return current != null ? current : DEFAULT;
    }

    public static boolean isSet() {
        return current != null;
    }

    public static void set(Alliance alliance) {
        current = alliance;
    }

    public static void clear() {
        current = null;
    }

    public static boolean isRed() {
        return get() == RED;
    }

    public static boolean isBlue() {
        return get() == BLUE;
    }


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
        if (this == BLUE) {
            return FieldConstants.BLUE_AIM_X;
        }
        return FieldConstants.AIM_POINT_INDEX == 1 ? FieldConstants.RED_AIM_X_1 : FieldConstants.RED_AIM_X;
    }

    /** 本方云台瞄准点 Y（Pedro 坐标系，英寸）。 */
    public double getAimY() {
        if (this == BLUE) {
            return FieldConstants.BLUE_AIM_Y;
        }
        return FieldConstants.AIM_POINT_INDEX == 1 ? FieldConstants.RED_AIM_Y_1 : FieldConstants.RED_AIM_Y;
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


    public Pose fromBlue(Pose bluePose) {
        return this == BLUE ? bluePose : mirror(bluePose);
    }

    /**
     * 把一个"按 {@link #RED} 写的" pose 转换到本联盟。
     */
    public Pose fromRed(Pose redPose) {
        return this == RED ? redPose : mirror(redPose);
    }

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
