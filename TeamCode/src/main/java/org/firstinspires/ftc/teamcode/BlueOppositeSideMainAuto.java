package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Opposite Side Main Auto", group = "Jimothy")
public class BlueOppositeSideMainAuto extends OppositeSideMainAutoBase {
    @Override
    protected String targetTagName() {
        return "BLUE AUDIENCE";
    }

    @Override
    protected boolean isTargetSingleTag(int id) {
        return id >= 42 && id <= 45;
    }
}
