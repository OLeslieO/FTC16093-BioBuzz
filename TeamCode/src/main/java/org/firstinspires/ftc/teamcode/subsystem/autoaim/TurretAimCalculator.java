package org.firstinspires.ftc.teamcode.subsystem.autoaim;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;


public final class TurretAimCalculator {

    private TurretAimCalculator() {
    }


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


    //SOTM stuff

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

    public static final class ShotSolution {
        public final double virtualX;
        public final double virtualY;
        public final double distance;
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

        for (int i = 0; i < Math.max(1, iterations); i++) {
            usedT = t;
            virtualX = targetX - vx * usedT;
            virtualY = targetY - vy * usedT;
            distance = Math.hypot(virtualX - releaseX, virtualY - releaseY);
            t = interpolate(flightDist, flightTime, distance, 0.0);
        }
        return new ShotSolution(virtualX, virtualY, distance, usedT);
    }


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

    public static double lockTolerance(double distance, double halfWidth, double minRad, double maxRad) {
        double tolerance = Math.atan2(halfWidth, Math.max(distance, 1e-6));
        return Math.max(minRad, Math.min(maxRad, tolerance));
    }
}
