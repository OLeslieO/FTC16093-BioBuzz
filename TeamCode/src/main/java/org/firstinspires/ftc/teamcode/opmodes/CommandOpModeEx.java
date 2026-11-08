package org.firstinspires.ftc.teamcode.opmodes;

import com.seattlesolvers.solverslib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {
    public abstract void onStart();

    public abstract void functionalButtons();

    /**
     * 按下 INIT 之后、START 之前每个循环调用一次。
     * 默认什么都不做；需要在 init 阶段读手柄（例如 {@code Alliance.selectWithGamepad(gamepad1)}）时覆盖。
     */
    public void initLoop() {
    }

    @Override
    public void runOpMode() {
        try {
            initialize();

            functionalButtons();

            telemetry.addLine("Ready!");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                initLoop();
                idle();
            }
            if (isStopRequested()) {
                return;
            }

            telemetry.clearAll();

            onStart();

            while (!isStopRequested() && opModeIsActive()) {
                run();
            }
        } finally {
            try {
                end();
            } finally {
                reset();
            }
        }
    }
}
