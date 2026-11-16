package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "cr servo test", group = "test")
@Configurable
public class CRServoTest extends LinearOpMode {

    // Set dualServo and servo names before pressing INIT.
    public static boolean dualServo = true;
    public static String servo_name_0 = "lala1";
    public static String servo_name_1 = "lala2";
    // Adjust live in Panels after START. Valid powers range from -1 to 1.
    public static double servo_power_0 = 0;
    public static double servo_power_1 = 0;
    // Only applied after START (re-read every loop).
    public static boolean reverse_0 = false;
    public static boolean reverse_1 = false;
    // When true, servo 1 follows servo 0's power instead of servo_power_1.
    public static boolean link_servos = false;

    @Override
    public void runOpMode() {
        CRServo servo0 = hardwareMap.get(CRServo.class, servo_name_0);
        CRServo[] servos = dualServo
                ? new CRServo[]{servo0, hardwareMap.get(CRServo.class, servo_name_1)}
                : new CRServo[]{servo0};
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        try {
            while (opModeIsActive()) {
                double[] targets = {
                        Range.clip(servo_power_0, -1, 1),
                        Range.clip(link_servos ? servo_power_0 : servo_power_1, -1, 1)
                };
                boolean[] reverse = {reverse_0, reverse_1};

                for (int i = 0; i < servos.length; i++) {
                    servos[i].setDirection(reverse[i]
                            ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                    servos[i].setPower(targets[i]);

                    // CR servos have no position feedback; this is the last commanded power.
                    panelsTelemetry.addData("servo_power_" + i, servos[i].getPower());
                }
                panelsTelemetry.update(telemetry);
                idle();
            }
        } finally {
            for (CRServo servo : servos) {
                servo.setPower(0);
            }
        }
    }
}
