package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Runs the red autonomous intake sequence from the TeleOp list so the manual
 * second-shooting path recorder is not cut off by the 30-second autonomous
 * period. The recorder itself waits indefinitely for Gamepad 1 B or Stop.
 */
@TeleOp(name = "Red Auto Path Recorder", group = "Jimothy Calibration")
public class RedAutoPathRecorderTeleOp extends BioBuzzForward30Auto {
    @Override
    protected boolean captureNextPositionAfterIntake() {
        return true;
    }
}
