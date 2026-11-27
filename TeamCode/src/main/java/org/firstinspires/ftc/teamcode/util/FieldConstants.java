package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class FieldConstants {

    public static final double FIELD_SIZE = 141.5;

    /** 镜像中线 */
    public static final double MIRROR_X = FIELD_SIZE / 2.0;

    // 蓝方云台瞄准点（Pedro 坐标系实测）
    public static double BLUE_AIM_X = 84.0;
    public static double BLUE_AIM_Y = 70.75;

    // 红方云台瞄准点（Pedro 坐标系实测）， 两个可切换：0 号为默认
    public static double RED_AIM_X = 57.7;
    public static double RED_AIM_Y = 53.0;
    public static double RED_AIM_X_1 = 57.7;
    public static double RED_AIM_Y_1 = 88.5;


    public static int AIM_POINT_INDEX = 0;

    public static double AIM_SPLIT_Y = FIELD_SIZE / 2.0;

    public static double AIM_SPLIT_HYSTERESIS_IN = 5.0;

    public static void updateAimPointForY(double robotY) {
        if (robotY > AIM_SPLIT_Y + AIM_SPLIT_HYSTERESIS_IN) {
            AIM_POINT_INDEX = 1;
        } else if (robotY < AIM_SPLIT_Y - AIM_SPLIT_HYSTERESIS_IN) {
            AIM_POINT_INDEX = 0;
        }
    }

    /** 根据 y 坐标直接选瞄准点（不带滞回），用于开局初始化。 */
    public static int aimPointIndexForY(double robotY) {
        return robotY > AIM_SPLIT_Y ? 1 : 0;
    }

    /** 在两个瞄准点之间切换。 */
    public static void toggleAimPoint() {
        AIM_POINT_INDEX = AIM_POINT_INDEX == 0 ? 1 : 0;
    }

    private FieldConstants() {
    }
}
