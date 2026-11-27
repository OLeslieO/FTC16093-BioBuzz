package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(2);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-7.893700787401576);
        c.yPodOffset.set(-1.9685039370078743);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0145386542382144);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.31885140295068283);
                Controller secondaryTranslationalForward = Controller.proportional(0.1251965391140355);
                Controller primaryTranslationalLateral = Controller.proportional(0.625073172684057);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23094783504397498);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01035566735637424));
                c.brake.set(Controller.proportionalFeedforward(0.007802317252918104));

                c.headingFeedback.set(Controller.proportional(4.926947559120933));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05880459864127016, 0.004118300546230415));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11285329984168892, 0.05378991331711312));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016720061707265182, 0.002040849081171043));

                c.maxAchievableForwardVelocity.set(84.80564801764335);
                c.maxAchievableStrafeVelocity.set(68.51783292113336);
                c.naturalForwardDeceleration.set(26.531808747829942);
                c.naturalStrafeDeceleration.set(66.81544305670825);
                //自定义
                c.brakeAggression.set(0.92);
            }
    );

    /** 用上面的底盘 / 定位 / 算法配置创建 Follower。 */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
