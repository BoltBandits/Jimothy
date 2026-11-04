package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red Auto Op Side route and mechanisms, triggered by the blue BioBuzz tags. */
@Autonomous(name = "Blue Auto Op Side", group = "Jimothy")
public class BlueAutoOpSide extends RedAutoOpSide {
    @Override
    protected String targetColorLabel() {
        return "BLUE";
    }

    @Override
    protected boolean isTargetCluster(String name) {
        return name.equalsIgnoreCase("BLUE SCORING")
                || name.equalsIgnoreCase("BLUE AUDIENCE");
    }

    @Override
    protected boolean isTargetSingleTag(int id) {
        return id >= 38 && id <= 45;
    }
}
