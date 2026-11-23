package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * OctoQuad 定位诊断。车放着不动，看是轮子、IMU 还是通信出了问题。
 *
 * <ul>
 *   <li>Pedro 的 OctoQuadLocalizer 负责写入参数（端口、offset、headingScalar），这里直接读 OctoQuad 原始数据</li>
 *   <li>A：重新校准 IMU 并清零位姿（车必须静止）</li>
 *   <li>B：清空统计（最大角速度、坏数据次数）</li>
 * </ul>
 *
 * <p>判断：
 * <ul>
 *   <li>Pod 原始读数不变，但 Heading 在变 → IMU 问题（安装方向 / 松动 / 校准）</li>
 *   <li>Pod 原始读数自己在变 → 轮子 / 编码器线 / 端口问题</li>
 *   <li>Bad CRC 或 Status 不是 RUNNING 的次数在涨 → I2C 通信问题（线、接口、干扰）</li>
 *   <li>Status 重新回到 WARMING_UP / CALIBRATING / NOT_INITIALIZED → OctoQuad 掉电重启了</li>
 * </ul>
 */
@TeleOp(name = "OctoQuad Diagnostic", group = "test")
public class OctoQuadDiagnostic extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // 只用来让 Pedro 按 Constants 配置 OctoQuad，这里不驱动底盘
        Follower follower = Constants.create(hardwareMap);
        OctoQuad oq = hardwareMap.get(OctoQuad.class, Constants.localizerConfig.name.get());
        int xPort = Constants.localizerConfig.xPodPort.get();
        int yPort = Constants.localizerConfig.yPodPort.get();

        OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
        OctoQuad.EncoderDataBlock enc = new OctoQuad.EncoderDataBlock();

        long reads = 0, badCrc = 0, notRunning = 0;
        double maxAbsRateDps = 0.0;
        OctoQuad.LocalizerStatus lastStatus = null;
        String statusChanges = "";
        boolean lastA = false, lastB = false;
        double lastTime = getRuntime();

        String fw = oq.getFirmwareVersionString();

        while (opModeInInit() || opModeIsActive()) {
            double now = getRuntime();
            double loopMs = (now - lastTime) * 1000.0;
            lastTime = now;

            if (gamepad1.a && !lastA) {
                oq.resetLocalizerAndCalibrateIMU();
                statusChanges = "";
            }
            if (gamepad1.b && !lastB) {
                reads = badCrc = notRunning = 0;
                maxAbsRateDps = 0.0;
                statusChanges = "";
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            oq.readLocalizerDataAndAllEncoderData(loc, enc);
            reads++;
            if (!loc.crcOk) badCrc++;
            if (loc.localizerStatus != OctoQuad.LocalizerStatus.RUNNING) notRunning++;
            if (loc.localizerStatus != lastStatus) {
                statusChanges = (statusChanges + " → " + loc.localizerStatus);
                if (statusChanges.length() > 120) {
                    statusChanges = statusChanges.substring(statusChanges.length() - 120);
                }
                lastStatus = loc.localizerStatus;
            }
            double rateDps = Math.toDegrees(loc.velHeading_radS);
            if (loc.isDataValid()) {
                maxAbsRateDps = Math.max(maxAbsRateDps, Math.abs(rateDps));
            }

            out.addData("Firmware", fw);
            out.addData("Status", loc.localizerStatus);
            out.addData("Status 变化", statusChanges);
            out.addData("Reads / Bad CRC / Not RUNNING", "%d / %d / %d", reads, badCrc, notRunning);
            out.addData("Heading", "%.2f°", Math.toDegrees(loc.heading_rad));
            out.addData("Heading Rate", "%.2f °/s", rateDps);
            out.addData("Max |Heading Rate|", "%.1f °/s", maxAbsRateDps);
            out.addData("Pos (mm)", "(%d, %d)", loc.posX_mm, loc.posY_mm);
            out.addData("Vel (mm/s)", "(%d, %d)", loc.velX_mmS, loc.velY_mmS);
            out.addData("xPod raw (port " + xPort + ")", "%d  vel %d", enc.positions[xPort], enc.velocities[xPort]);
            out.addData("yPod raw (port " + yPort + ")", "%d  vel %d", enc.positions[yPort], enc.velocities[yPort]);
            out.addData("Encoder CRC", enc.crcOk);
            out.addData("loop_ms", "%.1f", loopMs);
            out.addLine("A：重新校准 IMU（车别动）  B：清空统计");
            out.update();
        }
    }
}
