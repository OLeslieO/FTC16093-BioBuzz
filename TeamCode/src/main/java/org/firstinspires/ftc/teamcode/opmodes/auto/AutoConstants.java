package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class AutoConstants {

    public static class FieldPoint {
        public double x;
        public double y;
        public double h;

        public FieldPoint(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public Pose pose() {
            return pose(h);
        }

        public Pose pose(double headingDeg) {
            return POSE.of(x, y, headingDeg);
        }
    }

    private static final PoseFactory POSE = PoseFactory.degrees();

    // ================= 起点 =================

    /** 起点（下方）。 */
    public static FieldPoint START_BOT_VER = new FieldPoint(55.6, 6.7, 90);
    public static FieldPoint START_BOT_STRAFE = new FieldPoint(53.5,9.5,180);

    // ================= 中间一排 =================

    /** startToPrepmid 终点：中间一排吸球准备点。 */
    public static FieldPoint PREP_MID = new FieldPoint(22.13, 46.85, 180);
    /** prepmidToFlowermid 终点：中间一排吸球终点。 */
    public static FieldPoint FLOWER_MID = new FieldPoint(17.83, 46.5, 180);
    /** flowermidToPrepmid 终点：吸完退回的准备点。 */
    public static FieldPoint PREP_MID_2 = new FieldPoint(22.13, 46.85, 180);

    // ================= 射球点 =================

    /** prepmidToShoottop 终点：上方射球点。 */
    public static FieldPoint SHOOT_TOP = new FieldPoint(40.12, 117.09, 90);
    /** prepmidToShoottop 控制点。 */
    public static FieldPoint CTRL_PREP_MID_TO_SHOOT_TOP = new FieldPoint(25.8934, 118.0229, 0);
    /** loadbotToShoottop 终点：第二次上方射球点。 */
    public static FieldPoint SHOOT_TOP_2 = new FieldPoint(40.12, 117.09, -90);

    // ================= 上方一排 =================

    /** shoottopToPreptop 终点：上方吸球准备点。 */
    public static FieldPoint PREP_TOP = new FieldPoint(48.78,121.81, 90);
    /** preptopToFlowertop 终点：上方吸球终点。 */
    public static FieldPoint FLOWER_TOP = new FieldPoint(48.86, 126.81, 90);

    // ================= 下方装填区 =================
    public static FieldPoint CTRL_FLOWERTOP_PREPBOT_1 = new FieldPoint(24.5152, 127.5667, 0);
    public static FieldPoint CTRL_FLOWERTOP_PREPBOT_2 = new FieldPoint(33.8294, 26.9064, 0);



    /** flowertopToPrepbot 终点：装填区准备点。 */
    public static FieldPoint PREP_BOT = new FieldPoint(9.09, 20.59, 270);
    /** prepbotToLoadbot 终点：装填区。 */
    public static FieldPoint LOAD_BOT = new FieldPoint(9, 10.2, -90);

    // ================= 下方装填区 2（Full2TipV666） =================

    /** startToPrepbot2 终点：装填区 2 准备点。 */
    public static FieldPoint PREP_BOT_2 = new FieldPoint(20.428, 9.0927, 180);
    /** prepbot2ToLoadbot2 终点：装填区 2。 */
    public static FieldPoint LOAD_BOT_2 = new FieldPoint(10.124, 8.955, 180);

    // ================= Full2TipV666 控制点 =================

    /** loadbot2ToShoottop2 控制点。 */
    public static FieldPoint CTRL_LOAD_BOT_2_TO_SHOOT_TOP_2 = new FieldPoint(27.0976, 28.9747, 0);
    /** shoottop2ToFlowertop 控制点。 */
    public static FieldPoint CTRL_SHOOT_TOP_2_TO_FLOWER_TOP = new FieldPoint(48.149, 119.8617, 0);
    /** flowertopToShoottop2 控制点。 */
    public static FieldPoint CTRL_FLOWER_TOP_TO_SHOOT_TOP_2 = new FieldPoint(48.2024, 119.8973, 0);
    /** shoottop2ToPrepmid 控制点。 */
    public static FieldPoint CTRL_SHOOT_TOP_2_TO_PREP_MID = new FieldPoint(26.8992, 44.7676, 0);
    /** flowermidToShoottop2 控制点。 */
    public static FieldPoint CTRL_FLOWER_MID_TO_SHOOT_TOP_2 = new FieldPoint(29.9277, 53.1055, 0);

    // ================= 停车点 =================

    /** shoottopToPark 终点。 */
    public static FieldPoint PARK = new FieldPoint(12.44, 118.19205, 180);

    private AutoConstants() {
    }
}
