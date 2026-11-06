package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red Autonomous Mode route with a BLUE AUDIENCE gate before the second shot. */
@Autonomous(name = "Blue Auto Mode", group = "Jimothy")
public class BlueAutoMode extends BioBuzzForward30Auto {
    @Override
    protected String secondShotTargetName() {
        return "BLUE AUDIENCE";
    }

    @Override
    protected boolean isSecondShotTargetSingleTag(int id) {
        return id >= 42 && id <= 45;
    }
}
