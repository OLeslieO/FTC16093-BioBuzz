package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Shooter PID", group = "test")
@Configurable
public class ShooterPIDTest extends LinearOpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static boolean isPIDControl = true;
    public static double setP = 40;
    public static double setI = 0;
    public static double setD = 17;
    public static double setF = 17.5;
    public static double setShooterPower = 1;
    public static boolean isPowerMode = false;
    public static double setPreShooterPower = 1;
    //  public static double shooterMinVelocity = 1400.0;
    public static double shooterVelocity = 1440;

    public static volatile double servo_pos = 0.54;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooterPollen = hardwareMap.get(DcMotorEx.class, "shooterPollen");
        DcMotorEx shooterNectar = hardwareMap.get(DcMotorEx.class, "shooterNectar");


        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");

        Servo preLimit = hardwareMap.get(Servo.class,"preLimit");

        shooterPollen.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterNectar.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        preShooter.setDirection(DcMotorSimple.Direction.FORWARD);



        shooterPollen.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterNectar.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        shooterPollen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterPollen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterNectar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterNectar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (isPIDControl) {
            shooterPollen.setVelocityPIDFCoefficients(setP, setI, setD, setF);
            shooterNectar.setVelocityPIDFCoefficients(setP, setI, setD, setF);

        }

        waitForStart();

        while (opModeIsActive()) {
            if(isPowerMode){
                shooterPollen.setPower(setShooterPower);
                shooterNectar.setPower(setShooterPower);
            }
            else{
                shooterPollen.setVelocity(shooterVelocity);
                shooterNectar.setVelocity(shooterVelocity);


            }

//      if (frontShooter.getVelocity() > shooterMinVelocity) {
            if(gamepad1.a){

                intake.setPower(1);
                preShooter.setPower(setPreShooterPower);
                preLimit.setPosition(1);
            }
            else{

                intake.setPower(0);
                preShooter.setPower(0);
                preLimit.setPosition(0.68);
            }

//      if (frontShooter.getVelocity() < shooterMinVelocity) {
//        //            if(gamepad1.b){\
//        preShooter.setPower(0);
//        blender.setPower(0);
//        intake.setPower(0);
//      }
//      }

            panelsTelemetry.addData("Shooter Nectar Velocity", shooterNectar.getVelocity());
            panelsTelemetry.addData("Shooter Pollen Velocity", shooterPollen.getVelocity());

            panelsTelemetry.update(telemetry);

        }
    }
}