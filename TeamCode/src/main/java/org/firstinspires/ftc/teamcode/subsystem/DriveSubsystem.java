package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class DriveSubsystem extends SubsystemBase {
    private final DcMotorEx rightFront, leftFront, rightRear, leftRear;

    public DriveSubsystem(HardwareMap hardwareMap) {
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setBotCentric(double x, double y, double rx, double powerCoefficient) {
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator * powerCoefficient;
        double leftRearPower = (y - x + rx) / denominator * powerCoefficient;
        double rightFrontPower = (y - x - rx) / denominator * powerCoefficient;
        double rightRearPower = (y + x - rx) / denominator * powerCoefficient;

        setPower(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    public void setPower(double leftFrontPower, double leftRearPower,
                         double rightRearPower, double rightFrontPower) {
        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);
        rightRear.setPower(rightRearPower);
        leftRear.setPower(leftRearPower);
    }



    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
    public void stop() {
        setPower(0.0, 0.0,0.0,0.0);
    }

}
