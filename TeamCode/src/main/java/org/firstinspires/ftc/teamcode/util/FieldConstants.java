package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

/**
 * DECODE 场地常量，全部使用 Pedro Pathing 坐标系：
 * 单位英寸，原点在观众席视角的左下角，+X 向右（红方），+Y 远离观众席。
 * 红蓝双方关于 X = 72 这条中线镜像。
 *
 * 数值可以在 Panels 里实时调整。
 * 云台瞄准点为我们在 Pedro 坐标系中实测得到，上场前请在实际场地上核对。
 */
@Configurable
public class FieldConstants {

    /** 场地边长（英寸）。 */
    public static final double FIELD_SIZE = 141.5;

    /** 镜像中线，红蓝双方关于这条线对称。 */
    public static final double MIRROR_X = FIELD_SIZE / 2.0;

    // 蓝方云台瞄准点（Pedro 坐标系实测）
    public static double BLUE_AIM_X = 84.0;
    public static double BLUE_AIM_Y = 70.75;

    // 红方云台瞄准点（Pedro 坐标系实测），两个可切换：0 号为默认
    public static double RED_AIM_X = 59.0;
    public static double RED_AIM_Y = 59.0;
    public static double RED_AIM_X_1 = 59.0;
    public static double RED_AIM_Y_1 = 82.5;

    /**
     * 当前用哪个红方瞄准点：0 = (RED_AIM_X, RED_AIM_Y)，1 = (RED_AIM_X_1, RED_AIM_Y_1)。
     * 用 {@link #toggleAimPoint()} 切换；Panels 里也能直接改。
     */
    public static int AIM_POINT_INDEX = 0;

    /** 在两个瞄准点之间切换。 */
    public static void toggleAimPoint() {
        AIM_POINT_INDEX = AIM_POINT_INDEX == 0 ? 1 : 0;
    }

    private FieldConstants() {
    }
}
