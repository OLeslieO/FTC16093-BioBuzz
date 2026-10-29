package org.firstinspires.ftc.teamcode.opmodes;

import com.seattlesolvers.solverslib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {
    public abstract void onStart();

    public abstract void functionalButtons();

    @Override
    public void runOpMode() {
        try {
            initialize();

            functionalButtons();

            telemetry.addLine("Ready!");
            telemetry.update();

            waitForStart();
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
