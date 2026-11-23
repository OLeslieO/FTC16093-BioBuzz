package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Turret test: two CR servos on one gear, REV Through Bore on a motor port.
 *
 * Controller structure (single loop, no state machine):
 *
 *   1. A trapezoidal motion profile generates profilePos / profileVel
 *      that move smoothly from wherever the turret is to TARGET_DEG.
 *   2. power = kS * sign(profileVel)          static friction feedforward
 *            + kV * profileVel                 velocity feedforward
 *            + KP * (profilePos - angle)       position P (tracks the profile)
 *            + KD * (profileVel - vel)         damping
 *      ...while the profile is still moving.
 *   3. Once the profile has arrived, no P / I at all: inside TOL_DEG the
 *      power is 0 (the servo brakes); outside it a constant "nudge" just
 *      above breakaway pushes toward the target until it is inside
 *      TOL_INNER_DEG. See the Terminal nudge section for why.
 */
@TeleOp(name = "Turret Servo Test", group = "test")
@Configurable
public class TurretServoTest extends LinearOpMode {

    // ---- Hardware ----

    public static String SERVO_NAME_0 = "lala1";
    public static String SERVO_NAME_1 = "lala2";
    public static String ENCODER_NAME = "preShooter";

    public static boolean SERVO_REVERSED_0 = false;
    public static boolean SERVO_REVERSED_1 = false;
    public static boolean ENCODER_REVERSED = false;

    // REV Through Bore v1
    public static double ENCODER_CPR = 8192.0;

    // Encoder shaft revolutions / turret revolutions
    public static double ENCODER_TO_TURRET = 0.571;

    // ---- Modes ----
    // 0 = STOP
    // 1 = MANUAL POWER   (use this to measure kS and kV)
    // 2 = POSITION
    // 3 = STEP TEST

    public static int MODE = 0;

    public static double POWER = 0.1;
    public static double TARGET_DEG = 0.0;

    public static double STEP_A_DEG = -45.0;
    public static double STEP_B_DEG = 45.0;
    public static double STEP_PERIOD = 2.0;

    // ---- Soft Limits ----

    public static double MIN_ANGLE_DEG = -160.0;
    public static double MAX_ANGLE_DEG = 160.0;
    public static double LIMIT_MARGIN_DEG = 3.0;

    // ---- Motion Profile ----

    // Cruise speed of the reference. Start below what MODE 1 shows at
    // MAX_POWER so the servo can actually keep up.
    public static double MAX_VEL_DPS = 120.0;

    // Accel of the reference. 90 deg step at 120 deg/s, 400 deg/s^2
    // takes ~1.05 s.
    public static double MAX_ACCEL_DPS2 = 400.0;

    // ---- Feedforward (measure in MODE 1) ----

    // Smallest POWER that makes the turret move
    public static double kS = 0.13;

    // (POWER - kS) / steady filtered_vel_dps
    public static double kV = 0.002;

    // kS ramps in with |profileVel| over this speed, so it is continuous
    public static double KS_RAMP_VEL_DPS = 5.0;

    // ---- Feedback while tracking the profile ----

    // power / deg. 0.02 -> 5 deg of tracking error adds 0.10 power.
    public static double KP = 0.02;

    // power / (deg/s). Damping on the velocity tracking error.
    public static double KD = 0.0004;

    // ---- Terminal "nudge" once the profile has arrived ----
    //
    // A CR servo cannot creep: below kS it does not move at all, just
    // above kS it jumps to its minimum running speed. Any linear P / I
    // term therefore turns into a relay at kS (error builds -> jump ->
    // overshoot -> error builds the other way...). Lower KP only makes
    // the error needed to reach kS bigger and the ramp slower, which is
    // exactly "slower but larger" swinging.
    //
    // So near the target we do not use P at all:
    //   |error| <= TOL_DEG        -> power 0 (servo brakes), do nothing
    //   |error| >  TOL_DEG        -> nudge: constant power just above
    //                                breakaway toward the target until
    //                                |error| <= TOL_INNER_DEG or the
    //                                error changes sign, then power 0.
    // Hysteresis between TOL_DEG and TOL_INNER_DEG stops the ping-pong.
    // If it still bounces back and forth, RAISE TOL_DEG: it must be
    // larger than the overshoot of one nudge (min speed x delay).

    public static double TOL_DEG = 1.5;
    public static double TOL_INNER_DEG = 0.3;

    // Nudge power starts here and ramps up until motion is detected
    public static double NUDGE_START_POWER = 0.11;
    public static double NUDGE_RAMP_PER_S = 0.15;
    public static double NUDGE_MAX_POWER = 0.30;

    // |filteredVel| above this counts as "it broke free"
    public static double NUDGE_MOVING_VEL = 8.0;

    // Give up on a nudge after this long and restart the ramp
    public static double NUDGE_TIMEOUT_S = 1.5;

    // ---- Velocity Filter ----

    public static double VEL_FILTER_TAU = 0.03;

    public static double MAX_POWER = 0.6;

    // ---- Internal States ----

    private final TelemetryManager panels =
            PanelsTelemetry.INSTANCE.getTelemetry();

    private DcMotorEx encoder;

    private double ticksPerTurretRev;

    private double filteredVel = 0.0;

    private double profilePos = 0.0;
    private double profileVel = 0.0;

    // Terminal nudge state
    private boolean nudging = false;
    private double nudgeSign = 0.0;
    private double nudgePower = 0.0;
    private double nudgeStart = 0.0;
    private boolean nudgeBrokeFree = false;

    // Last power that actually broke the turret free (adaptive kS)
    private double learnedBreakaway = Double.NaN;

    private int previousMode = -1;

    @Override
    public void runOpMode() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CRServo[] servos = {
                hardwareMap.get(CRServo.class, SERVO_NAME_0),
                hardwareMap.get(CRServo.class, SERVO_NAME_1)
        };

        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0.0);

        ticksPerTurretRev = ENCODER_CPR / ENCODER_TO_TURRET;

        // ---- INIT ----

        while (opModeInInit()) {
            panels.addData("ticks / turret rev", ticksPerTurretRev);
            panels.addData("angle_deg", angleDeg());
            panels.addLine("向左转动云台，angle_deg 应增加");
            panels.update(telemetry);
        }

        if (isStopRequested()) {
            return;
        }

        double lastTime = getRuntime();
        double stepStart = getRuntime();
        boolean stepAtB = false;

        try {

            while (opModeIsActive()) {

                double now = getRuntime();
                double dt = now - lastTime;
                lastTime = now;

                double controlDt = clip(dt, 0.001, 0.05);

                // ---- Servo Direction ----

                servos[0].setDirection(SERVO_REVERSED_0
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);
                servos[1].setDirection(SERVO_REVERSED_1
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);

                // ---- Sensors ----

                double angle = angleDeg();
                double rawVel = velocityDps();

                double alpha = controlDt / (VEL_FILTER_TAU + controlDt);
                filteredVel += alpha * (rawVel - filteredVel);

                // ---- Per-loop outputs for telemetry ----

                double target = Double.NaN;
                double error = 0.0;
                double ffPower = 0.0;
                double pPower = 0.0;
                double dPower = 0.0;
                double power = 0.0;
                boolean arrived = false;
                String state = "-";

                // ---- Mode Transition ----

                if (MODE != previousMode) {

                    previousMode = MODE;

                    // Start the profile from where the turret actually is
                    profilePos = angle;
                    profileVel = 0.0;
                    nudging = false;

                    if (MODE == 3) {
                        stepStart = now;
                        stepAtB = false;
                    }
                }

                // ---- MODE 1: Manual ----

                if (MODE == 1) {

                    power = POWER;
                }

                // ---- MODE 2 / 3: Position ----

                else if (MODE == 2 || MODE == 3) {

                    // Target

                    if (MODE == 3) {
                        if (now - stepStart >= STEP_PERIOD) {
                            stepAtB = !stepAtB;
                            stepStart = now;
                        }
                        target = stepAtB ? STEP_B_DEG : STEP_A_DEG;
                    } else {
                        target = TARGET_DEG;
                    }

                    target = clip(target,
                            MIN_ANGLE_DEG + LIMIT_MARGIN_DEG,
                            MAX_ANGLE_DEG - LIMIT_MARGIN_DEG);

                    // ---- Trapezoidal profile ----
                    // Move the reference toward the target at a speed
                    // that lets it stop exactly there with MAX_ACCEL.

                    double remaining = target - profilePos;

                    double stoppingSpeed = Math.sqrt(
                            2.0 * MAX_ACCEL_DPS2 * Math.abs(remaining));

                    double desiredVel = Math.copySign(
                            Math.min(MAX_VEL_DPS, stoppingSpeed),
                            remaining);

                    profileVel = moveTowards(profileVel, desiredVel,
                            MAX_ACCEL_DPS2 * controlDt);

                    profilePos += profileVel * controlDt;

                    // Snap when the reference is essentially there
                    if (Math.abs(target - profilePos) < 0.05
                            && Math.abs(profileVel) < 1.0) {
                        profilePos = target;
                        profileVel = 0.0;
                    }

                    arrived = profilePos == target && profileVel == 0.0;

                    // ---- Errors ----

                    error = profilePos - angle;

                    double targetError = target - angle;
                    double absTargetError = Math.abs(targetError);

                    if (!arrived) {

                        // ---- Tracking the profile: FF + PD ----

                        nudging = false;

                        double ksScale = clip(
                                Math.abs(profileVel) / KS_RAMP_VEL_DPS,
                                0.0, 1.0);

                        ffPower = Math.copySign(kS * ksScale, profileVel)
                                + kV * profileVel;

                        pPower = KP * error;

                        dPower = KD * (profileVel - filteredVel);

                        power = ffPower + pPower + dPower;

                        state = "MOVING";

                    } else {

                        // ---- Arrived: nudge with hysteresis ----

                        if (nudging) {

                            boolean reached =
                                    absTargetError <= TOL_INNER_DEG
                                            || targetError * nudgeSign < 0;

                            boolean timedOut =
                                    now - nudgeStart > NUDGE_TIMEOUT_S;

                            if (reached || timedOut) {

                                nudging = false;

                            } else {

                                if (!nudgeBrokeFree) {

                                    if (Math.abs(filteredVel)
                                            > NUDGE_MOVING_VEL) {

                                        nudgeBrokeFree = true;
                                        learnedBreakaway = nudgePower;

                                    } else {

                                        nudgePower = Math.min(
                                                NUDGE_MAX_POWER,
                                                nudgePower
                                                        + NUDGE_RAMP_PER_S
                                                        * controlDt);
                                    }
                                }

                                power = nudgeSign * nudgePower;
                            }
                        }

                        if (!nudging) {

                            if (absTargetError > TOL_DEG) {

                                nudging = true;
                                nudgeSign = Math.signum(targetError);
                                nudgeStart = now;
                                nudgeBrokeFree = false;

                                nudgePower = Double.isNaN(learnedBreakaway)
                                        ? NUDGE_START_POWER
                                        : Math.max(NUDGE_START_POWER,
                                        0.9 * learnedBreakaway);

                                power = nudgeSign * nudgePower;

                            } else {

                                // Inside tolerance: 0 power, servo brakes
                                power = 0.0;
                            }
                        }

                        state = nudging ? "NUDGE" : "HOLD";
                    }
                }

                // ---- MODE 0: STOP ----

                else {
                    power = 0.0;
                }

                // ---- Output Limits ----

                power = clip(power, -MAX_POWER, MAX_POWER);
                power = applyLimits(power, angle);

                for (CRServo servo : servos) {
                    servo.setPower(power);
                }

                // ---- Telemetry ----

                panels.addData("MODE", MODE);
                panels.addData("STATE", state);

                if (!Double.isNaN(target)) {
                    panels.addData("target_deg", target);
                    panels.addData("profile_pos_deg", profilePos);
                    panels.addData("profile_vel_dps", profileVel);
                    panels.addData("track_error_deg", error);
                    panels.addData("target_error_deg", target - angle);
                }

                panels.addData("angle_deg", angle);
                panels.addData("raw_vel_dps", rawVel);
                panels.addData("filtered_vel_dps", filteredVel);

                panels.addData("ff_power", ffPower);
                panels.addData("p_power", pPower);
                panels.addData("d_power", dPower);
                panels.addData("nudge_power", nudging ? nudgePower : 0.0);
                panels.addData("learned_breakaway", learnedBreakaway);
                panels.addData("power", power);

                panels.addData("encoder_ticks", rawTicks());
                panels.addData("loop_ms", dt * 1000.0);

                panels.update(telemetry);
            }

        } finally {
            for (CRServo servo : servos) {
                servo.setPower(0.0);
            }
        }
    }

    // ---- Motion Profile Helper ----

    private static double moveTowards(double current, double target,
                                      double maxDelta) {
        if (Math.abs(target - current) <= maxDelta) {
            return target;
        }
        return current + Math.copySign(maxDelta, target - current);
    }

    // ---- Encoder ----

    private int rawTicks() {
        return encoder.getCurrentPosition();
    }

    private double sign() {
        return ENCODER_REVERSED ? -1.0 : 1.0;
    }

    private double angleDeg() {
        return sign() * rawTicks() / ticksPerTurretRev * 360.0;
    }

    private double velocityDps() {
        return sign() * encoder.getVelocity() / ticksPerTurretRev * 360.0;
    }

    // ---- Soft Limits ----

    private static double applyLimits(double power, double angleDeg) {
        if (power > 0.0 && angleDeg >= MAX_ANGLE_DEG - LIMIT_MARGIN_DEG) {
            return 0.0;
        }
        if (power < 0.0 && angleDeg <= MIN_ANGLE_DEG + LIMIT_MARGIN_DEG) {
            return 0.0;
        }
        return power;
    }

    // ---- Utility ----

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
