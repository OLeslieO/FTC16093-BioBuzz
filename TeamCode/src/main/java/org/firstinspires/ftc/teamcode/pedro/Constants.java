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
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(7.6);
        c.yPodOffset.set(1.0);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0130827808576748);
    });



    /**
     * 路径跟随算法参数。
     *
     * <p>TODO: 还没调。跑 Tuning 里的 ForesightTuner，把它输出的
     * {@code public static ForesightConfig foresightConfig = ...} 粘贴到这里，
     * 然后把 {@link #create} 里的 {@code foresightConfig()} 换成那个字段。
     * 未调参前 ForesightConfig 的 required 项为空，创建时会抛异常——
     * 放在方法里而不是静态字段，是为了不影响 Tuning 加载这个类。
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.31088990025214064);
                Controller secondaryTranslationalForward = Controller.proportional(0.11486551101203661);
                Controller primaryTranslationalLateral = Controller.proportional(0.38670997637525373);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14287900318978045);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012366682978628673));
                c.brake.set(Controller.proportionalFeedforward(0.010511680531834372));

                c.headingFeedback.set(Controller.proportional(4.714047951623178));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.051480699529798925, 0.007749251586769882));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07513759701663385, 0.06448955055727004));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018641733911233956, 0.0018797615842739929));

                c.maxAchievableForwardVelocity.set(72.87816719309397);
                c.maxAchievableStrafeVelocity.set(60.97195843883922);
                c.naturalForwardDeceleration.set(28.967602254533556);
                c.naturalStrafeDeceleration.set(60.49596529406293);
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
