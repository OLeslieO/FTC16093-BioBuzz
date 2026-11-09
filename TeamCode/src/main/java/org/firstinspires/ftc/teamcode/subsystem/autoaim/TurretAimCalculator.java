package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

/**
 * 云台瞄准的纯数学部分，不碰硬件，方便单独验证。
 * 角度单位全部是弧度，逆时针为正，云台角度 0 = 机器人正前方。
 */
public final class TurretAimCalculator {

    private TurretAimCalculator() {
    }

    /**
     * 云台旋转轴在场地上的位置：机器人位置 + 按 heading 旋转后的偏移。
     *
     * @param forwardIn 旋转轴在机器人中心前方多少英寸
     * @param leftIn    旋转轴在机器人中心左方多少英寸
     */
    public static Pose pivotPose(Pose robot, double forwardIn, double leftIn) {
        double cos = Math.cos(robot.heading());
        double sin = Math.sin(robot.heading());
        return new Pose(
                robot.x() + forwardIn * cos - leftIn * sin,
                robot.y() + forwardIn * sin + leftIn * cos,
                robot.heading()
        );
    }

    /** 从旋转轴指向瞄准点的场地绝对角度，范围 (-π, π]。 */
    public static double fieldAngleToAim(Pose pivot, Pose aim) {
        return Math.atan2(aim.y() - pivot.y(), aim.x() - pivot.x());
    }

    /**
     * 在机械范围 [min, max] 内选一个能指向 {@code desired} 方向的云台角度。
     *
     * <p>同一个方向有多个等价角度（差 2π 的整数倍），选离当前角度最近的，
     * 这样云台不会无故绕远路。如果这个方向落在机械死区里，就取离它最近的那个限位。
     *
     * @param desired 想要的云台相对角（任意范围）
     * @param current 云台当前角度
     */
    public static double chooseTurretAngle(double desired, double current, double min, double max) {
        double base = Angle.normalizeSigned(desired);

        double best = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double candidate = base + k * 2.0 * Math.PI;
            if (candidate < min || candidate > max) {
                continue;
            }
            double distance = Math.abs(candidate - current);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = candidate;
            }
        }
        if (!Double.isNaN(best)) {
            return best;
        }

        double toMin = Math.abs(Angle.normalizeSigned(desired - min));
        double toMax = Math.abs(Angle.normalizeSigned(desired - max));
        return toMin < toMax ? min : max;
    }
}
