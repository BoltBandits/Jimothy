package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Runs the current intake path, then allows push-to-position pose capture. */
@Autonomous(name = "Jimothy Next Position Test", group = "Jimothy Tests")
public class JimothyNextPositionTest extends BioBuzzForward30Auto {
    @Override
    protected boolean captureNextPositionAfterIntake() {
        return true;
    }
}
