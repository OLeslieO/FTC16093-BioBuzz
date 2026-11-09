package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "motor test", group = "test")
@Configurable
public class MotorTest extends LinearOpMode {

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static int encoder_position = 1150;
    public static double max_power = 1;
    public static int shooterVelocity = 1000;
    public static boolean read_only = false;
    public static boolean reverse_0 = false;
    public static boolean reverse_1 = false;
    public static boolean reset = true;
    public static boolean set_power_mode = false;
    public static boolean set_velocity_mode = true;
    // Mode priority: power, velocity, position. Position mode only drives motor0.
    public static boolean set_position_mode = false;
    // Set dualMotor and motor names before pressing INIT.
    public static boolean dualMotor = false;
    public static String motor_name_0 = "shooterUp";
    public static String motor_name_1 = "shooterDown";

    @Override
    public void runOpMode() {
        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, motor_name_0);
        DcMotorEx[] motors = dualMotor
                ? new DcMotorEx[]{motor0, hardwareMap.get(DcMotorEx.class, motor_name_1)}
                : new DcMotorEx[]{motor0};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        for (int i = 0; i < motors.length; i++) {
            if (reset) {
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            boolean reverse = i == 0 ? reverse_0 : reverse_1;
            motors[i].setDirection(reverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        try {
            while (opModeIsActive()) {
                if (read_only) {
                    for (DcMotorEx motor : motors) {
                        motor.setPower(0);
                    }
                } else if (set_power_mode) {
                    for (DcMotorEx motor : motors) {
                        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        motor.setPower(max_power);
                    }
                } else if (set_velocity_mode) {
                    for (DcMotorEx motor : motors) {
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motor.setVelocity(shooterVelocity);
                    }
                } else if (set_position_mode) {
                    for (int i = 1; i < motors.length; i++) {
                        motors[i].setPower(0);
                    }
                    motor0.setTargetPosition(encoder_position);
                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor0.setPower(Math.abs(max_power));
                    panelsTelemetry.addData("position_target", encoder_position);
                } else {
                    for (DcMotorEx motor : motors) {
                        motor.setPower(0);
                    }
                }
                for (int i = 0; i < motors.length; i++) {
                    panelsTelemetry.addData("is busy_" + i, motors[i].isBusy());
                    panelsTelemetry.addData("encoder_" + i, motors[i].getCurrentPosition());
                    panelsTelemetry.addData("velocity_" + i, motors[i].getVelocity());
                }
                panelsTelemetry.update(telemetry);
                idle();
            }
        } finally {
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
        }
    }
}
