package org.firstinspires.ftc.teamcode.opmodes;

import com.seattlesolvers.solverslib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {
    public abstract void onStart();

    public abstract void functionalButtons();


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
