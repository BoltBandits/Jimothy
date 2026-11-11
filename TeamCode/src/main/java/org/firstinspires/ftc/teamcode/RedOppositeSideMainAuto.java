package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Opposite Side Main Auto", group = "Jimothy")
public class RedOppositeSideMainAuto extends OppositeSideMainAutoBase {
    @Override
    protected String targetTagName() {
        return "RED SCORING";
    }

    @Override
    protected boolean isTargetSingleTag(int id) {
        return id >= 30 && id <= 33;
    }

    @Override
    protected boolean driveToRecordedNextPositionAfterShooting() {
        return true;
    }
}
