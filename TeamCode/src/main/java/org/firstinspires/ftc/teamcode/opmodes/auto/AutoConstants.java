package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * 所有自动程序共用的点位（英寸 / 度，Pedro Visualizer 坐标，红方）。
 *
 * <p>同一个位置只写一次，几个自动都引用它，在 Panels 里改一处就全部生效。
 * 同一位置在不同路径里朝向不同时，{@code h} 是默认朝向，其它朝向在自动里用 {@link FieldPoint#pose(double)} 指定。
 *
 * <p>自动在 START 时才读这些值，所以 INIT 之后在 Panels 里改也来得及。
 * 控制点（CTRL_*）只用 x / y，朝向无意义。
 */
@Configurable
public class AutoConstants {

    /** 一个可在 Panels 里调的点：x、y（英寸），h（度）。 */
    public static class FieldPoint {
        public double x;
        public double y;
        public double h;

        public FieldPoint(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        /** 用默认朝向 h。 */
        public Pose pose() {
            return pose(h);
        }

        /** 同一位置，换个朝向（度）。 */
        public Pose pose(double headingDeg) {
            return POSE.of(x, y, headingDeg);
        }
    }

    private static final PoseFactory POSE = PoseFactory.degrees();

    // ================= 起点 =================

    /** 下方起点：RedFull / RedFullPark / RedHalfParkBottom。 */
    public static FieldPoint START_BOTTOM = new FieldPoint(55.64, 7.677, 90);
    /** 上方起点：RedHalfTop。 */
    public static FieldPoint START_TOP = new FieldPoint(31.8, 133.823, -90);
    /** LeavePark 起点。 */
    public static FieldPoint LEAVE_START = new FieldPoint(10.87, 136.73, -90);

    // ================= 射球点 =================

    /** 下方射球点：朝向 90（开局）/ 180 / -90 都有用到。 */
    public static FieldPoint SHOOT_BOTTOM = new FieldPoint(55.84, 21.6, 90);
    /** 上方射球点：朝向 180 / -90 都有用到。 */
    public static FieldPoint SHOOT_TOP = new FieldPoint(55.84, 118, 180);

    // ================= 吸球点 =================

    /** 中间一排吸球准备点。 */
    public static FieldPoint POLLEN_MID_PREP = new FieldPoint(26.57, 44.09, 180);
    /** 中间一排吸球终点。 */
    public static FieldPoint POLLEN_MID = new FieldPoint(18.62, 44.09, 180);

    /** 上方吸球准备点（RedFull / RedFullPark / RedHalfTop）。 */
    public static FieldPoint POLLEN_TOP_PREP = new FieldPoint(43.27, 119.37, 90);
    /** 上方吸球终点。 */
    public static FieldPoint POLLEN_TOP = new FieldPoint(43.27, 124.96, 90);

    /** 下方吸球准备点（RedHalfParkBottom）。 */
    public static FieldPoint POLLEN_BOTTOM_PREP = new FieldPoint(23.5, 8.9, 180);
    /** 下方吸球终点（RedHalfParkBottom）。 */
    public static FieldPoint POLLEN_BOTTOM = new FieldPoint(9.8283, 8.9, 180);

    /** 左下角吸球点（RedFull 第三次吸球）。 */
    public static FieldPoint POLLEN_CORNER_PREP = new FieldPoint(9.02, 25.24, -90);

    public static FieldPoint POLLEN_CORNER = new FieldPoint(8.14, 15.2, -90);

    // ================= 停车点 =================

    /** 上方停车点：RedFullPark / RedHalfTop。 */
    public static FieldPoint PARK_TOP = new FieldPoint(20.63, 120.67, 180);
    /** RedHalfParkBottom 停车点。 */
    public static FieldPoint PARK_MID = new FieldPoint(10.0985, 92.0653, 90);
    /** LeavePark 终点。 */
    public static FieldPoint LEAVE_END = new FieldPoint(10.55, 128.07, -90);

    // ================= 曲线控制点 =================

    /** START_TOP → SHOOT_TOP（RedHalfTop）。 */
    public static FieldPoint CTRL_START_TOP_TO_SHOOT_TOP = new FieldPoint(31.466, 119.2664, 0);
    /** POLLEN_MID → SHOOT_TOP（RedFull / RedFullPark）。 */
    public static FieldPoint CTRL_MID_TO_SHOOT_TOP = new FieldPoint(23.2677, 118.5011, 0);
    /** POLLEN_MID → SHOOT_BOTTOM（RedHalfParkBottom）。 */
    public static FieldPoint CTRL_MID_TO_SHOOT_BOTTOM = new FieldPoint(39.5681, 27.1593, 0);
    /** POLLEN_TOP → SHOOT_TOP（RedFull / RedFullPark / RedHalfTop）。 */
    public static FieldPoint CTRL_TOP_TO_SHOOT_TOP = new FieldPoint(46.9869, 118.4687, 0);
    /** SHOOT_TOP → POLLEN_CORNER 第一个控制点（RedFull）。 */
    public static FieldPoint CTRL_SHOOT_TOP_TO_CORNER_1 = new FieldPoint(16.8889, 118.9597, 0);
    /** SHOOT_TOP → POLLEN_CORNER 第二个控制点（RedFull）。 */
    public static FieldPoint CTRL_SHOOT_TOP_TO_CORNER_2 = new FieldPoint(22.708, 22.5241, 0);
    /** SHOOT_BOTTOM → PARK_MID 第一个控制点（RedHalfParkBottom）。 */
    public static FieldPoint CTRL_PARK_MID_1 = new FieldPoint(8.7652, 47.0611, 0);
    /** SHOOT_BOTTOM → PARK_MID 第二个控制点（RedHalfParkBottom）。 */
    public static FieldPoint CTRL_PARK_MID_2 = new FieldPoint(27.7804, 80.5219, 0);

    private AutoConstants() {
    }
}
