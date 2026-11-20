package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Upgraded TeleOp whose alignment light recognizes blue scoring and audience tags. */
@TeleOp(name = "Jimothy Blue Upgraded TeleOp", group = "Jimothy")
public class JimothyBlueUpgradedTeleOp extends JimothyUpgradedTeleOp {
    @Override
    protected String targetAllianceName() {
        return "BLUE";
    }

    @Override
    protected String targetTagDescription() {
        return "BLUE SCORING / BLUE AUDIENCE";
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

    @Override
    protected String targetNameForSingleTag(int id) {
        return id <= 41 ? "BLUE AUDIENCE" : "BLUE SCORING";
    }

    @Override
    protected String controlsDescription() {
        return "BLUE UPGRADED: RB 45%; D-pad U/R/D/L shoots 48/51/55/57%";
    }
}
