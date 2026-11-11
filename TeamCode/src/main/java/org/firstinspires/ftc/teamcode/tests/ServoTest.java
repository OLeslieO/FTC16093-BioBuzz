package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "servo test", group = "test")
@Configurable
public class ServoTest extends LinearOpMode {

    // Set dualServo and servo names before pressing INIT.
    public static boolean dualServo = true;
    public static String servo_name_0 = "leftIntakeServo";
    public static String servo_name_1 = "rightIntakeServo";
    // Adjust live in Panels after START. Valid positions range from 0 to 1.
    public static double servo_position_0 = 0.5;
    public static double servo_position_1 = 0.5;
    // Only applied after START (re-read every loop).
    public static boolean reverse_0 = false;
    public static boolean reverse_1 = false;
    // When true, servo 1 follows servo 0's position instead of servo_position_1.
    public static boolean link_servos = false;

    @Override
    public void runOpMode() {
        Servo servo0 = hardwareMap.get(Servo.class, servo_name_0);
        Servo[] servos = dualServo
                ? new Servo[]{servo0, hardwareMap.get(Servo.class, servo_name_1)}
                : new Servo[]{servo0};
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            double[] targets = {
                    Range.clip(servo_position_0, 0, 1),
                    Range.clip(link_servos ? servo_position_0 : servo_position_1, 0, 1)
            };
            boolean[] reverse = {reverse_0, reverse_1};

            for (int i = 0; i < servos.length; i++) {
                servos[i].setDirection(reverse[i] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                servos[i].setPosition(targets[i]);

                // Standard servos report the last command, not a measured physical position.
                panelsTelemetry.addData("servo_position_" + i, servos[i].getPosition());
            }
            panelsTelemetry.update(telemetry);
            idle();
        }
    }
}
