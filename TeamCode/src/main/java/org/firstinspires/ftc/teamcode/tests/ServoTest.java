package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "servo test", group = "test")
@Config
public class ServoTest extends LinearOpMode {

    // Set the hardware name before pressing INIT.
    public static String servo_name = "hood";
    // Adjust live in Dashboard after START. Valid positions range from 0 to 1.
    public static double servo_position = 0.5;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, servo_name);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            double targetPosition = servo_position;

            servo.setPosition(Range.clip(targetPosition, 0, 1));

            // Standard servos report the last command, not a measured physical position.
            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();
            idle();
        }
    }
}
