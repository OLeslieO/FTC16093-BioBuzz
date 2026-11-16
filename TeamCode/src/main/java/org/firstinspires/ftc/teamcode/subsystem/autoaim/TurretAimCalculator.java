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

    // ---------------------------------------------------------------
    // 边走边射
    // ---------------------------------------------------------------

    /**
     * 按距离线性插值查表。超出表的范围时取两端的值。
     * 表格无效（为空 / 长度不同）时返回 {@code fallback}。
     *
     * @param xs 自变量，必须递增
     */
    public static double interpolate(double[] xs, double[] ys, double x, double fallback) {
        if (xs == null || ys == null || xs.length == 0 || xs.length != ys.length) {
            return fallback;
        }
        if (x <= xs[0]) {
            return ys[0];
        }
        int last = xs.length - 1;
        if (x >= xs[last]) {
            return ys[last];
        }
        for (int i = 0; i < last; i++) {
            if (x <= xs[i + 1]) {
                double span = xs[i + 1] - xs[i];
                if (span <= 0.0) {
                    return ys[i + 1];
                }
                return ys[i] + (x - xs[i]) / span * (ys[i + 1] - ys[i]);
            }
        }
        return ys[last];
    }

    /** 一次边走边射的解。 */
    public static final class ShotSolution {
        /** 虚拟目标：云台应该瞄准的场地坐标。 */
        public final double virtualX;
        public final double virtualY;
        /** 出射点到虚拟目标的距离，射速 / 俯仰应按这个距离查表。 */
        public final double distance;
        /** 球的滞空时间（秒）。 */
        public final double flightTime;

        ShotSolution(double virtualX, double virtualY, double distance, double flightTime) {
            this.virtualX = virtualX;
            this.virtualY = virtualY;
            this.distance = distance;
            this.flightTime = flightTime;
        }
    }

    /**
     * 边走边射的提前量。
     *
     * <p>球离开时会继承出射点的速度 {@code v}，飞行 {@code t} 秒后多走了 {@code v·t}。
     * 所以要瞄准"虚拟目标" = 真实目标 − v·t。{@code t} 又取决于到虚拟目标的距离，
     * 迭代几次就能收敛。速度为 0 时虚拟目标就是真实目标。
     *
     * @param releaseX  出射时旋转轴的场地位置
     * @param vx        出射时旋转轴的场地速度（已乘过继承比例）
     */
    public static ShotSolution solveMovingShot(
            double releaseX, double releaseY,
            double vx, double vy,
            double targetX, double targetY,
            double[] flightDist, double[] flightTime,
            int iterations
    ) {
        double t = interpolate(flightDist, flightTime,
                Math.hypot(targetX - releaseX, targetY - releaseY), 0.0);
        double usedT = t;
        double virtualX = targetX;
        double virtualY = targetY;
        double distance = 0.0;

        // 返回的虚拟目标、距离、滞空时间三者互相对应：virtual = target − v·usedT
        for (int i = 0; i < Math.max(1, iterations); i++) {
            usedT = t;
            virtualX = targetX - vx * usedT;
            virtualY = targetY - vy * usedT;
            distance = Math.hypot(virtualX - releaseX, virtualY - releaseY);
            t = interpolate(flightDist, flightTime, distance, 0.0);
        }
        return new ShotSolution(virtualX, virtualY, distance, usedT);
    }

    /**
     * 视线角速度（rad/s）：旋转轴以速度 v 平移时，指向固定点的场地角度的变化率。
     * 用作云台的速度前馈。
     */
    public static double lineOfSightRate(
            double pivotX, double pivotY,
            double vx, double vy,
            double targetX, double targetY
    ) {
        double dx = targetX - pivotX;
        double dy = targetY - pivotY;
        double distanceSq = dx * dx + dy * dy;
        if (distanceSq < 1e-6) {
            return 0.0;
        }
        // d/dt atan2(dy, dx)，其中 d(dx)/dt = -vx，d(dy)/dt = -vy
        return (dy * vx - dx * vy) / distanceSq;
    }

    /** 锁定容差角（弧度）：atan(目标半宽 / 距离)，再夹到上下限之间。 */
    public static double lockTolerance(double distance, double halfWidth, double minRad, double maxRad) {
        double tolerance = Math.atan2(halfWidth, Math.max(distance, 1e-6));
        return Math.max(minRad, Math.min(maxRad, tolerance));
    }
}
