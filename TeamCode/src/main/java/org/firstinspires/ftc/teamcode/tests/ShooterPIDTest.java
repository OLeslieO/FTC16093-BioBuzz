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
    public static double setP = 34;
    public static double setI = 0;
    public static double setD = 0;
    public static double setF = 16.5;
    public static double setShooterPower = 1;
    public static boolean isPowerMode = false;
    public static double setPreShooterPower = 1;
    //  public static double shooterMinVelocity = 1400.0;
    public static double shooterVelocity = 1640;
    public static boolean isPollen = true;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooterPollen = hardwareMap.get(DcMotorEx.class, "PollenShooter");
        DcMotorEx shooterNectar = hardwareMap.get(DcMotorEx.class, "NectarShooter");
        Servo pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        Servo nectarGate = hardwareMap.get(Servo.class, "nectarGate");



        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake1");


        shooterPollen.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterNectar.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);



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
            if(isPollen){
                shooterPollen.setVelocity(shooterVelocity);

            }
            else{
                shooterNectar.setVelocity(shooterVelocity);



            }

//      if (frontShooter.getVelocity() > shooterMinVelocity) {
            if(gamepad1.a){

                intake.setPower(1);
                intake1.setPower(1);
            }
            else{

                intake.setPower(0);
                intake1.setPower(0);
            }
            if(gamepad1.right_bumper){
                nectarGate.setPosition(0.5);
                pollenGate.setPosition(0.5);
            }else {
                nectarGate.setPosition(1);
                pollenGate.setPosition(1);
            }


//      if (frontShooter.getVelocity() < shooterMinVelocity) {
//        //            if(gamepad1.b){\
//        preShooter.setPower(0);
//        blender.setPower(0);
//        intake.setPower(0);
//      }
//      }

            // Panels Graph 只吃 telemetry 里的数值：目标和实际用同一个量纲（ticks/s）才能画在一起。
            // 功率模式下没有目标转速，记 0。
            double targetVelocity = isPowerMode ? 0.0 : shooterVelocity;
            double pollenVelocity = shooterPollen.getVelocity();
            panelsTelemetry.addData("Shooter Target Velocity", targetVelocity);
            panelsTelemetry.addData("Shooter Pollen Velocity", pollenVelocity);
            panelsTelemetry.addData("Shooter Velocity Error", targetVelocity - pollenVelocity);
            panelsTelemetry.addData("Shooter Nectar Velocity", shooterNectar.getVelocity());

            panelsTelemetry.update(telemetry);

        }
    }
}