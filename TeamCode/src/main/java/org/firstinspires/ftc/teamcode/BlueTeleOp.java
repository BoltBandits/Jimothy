package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Blue-side mirror of Red TeleOp's vision, shooter, and aim calibration. */
@TeleOp(name = "Blue Teleop", group = "Jimothy")
public class BlueTeleOp extends RedTeleOp {
    @Override
    protected String teleOpAllianceName() {
        return "Blue";
    }

    @Override
    protected String defaultTargetName() {
        return "BLUE SCORING";
    }

    @Override
    protected boolean isTeleOpTargetCluster(String name) {
        return name.equalsIgnoreCase("BLUE SCORING")
                || name.equalsIgnoreCase("BLUE AUDIENCE");
    }

    @Override
    protected boolean isMirroredTargetCluster(String name) {
        // The complete blue half is the field-space reflection of red.
        return name.equalsIgnoreCase("BLUE SCORING");
    }

    @Override
    protected boolean isTeleOpTargetSingleTag(int id) {
        return id >= 38 && id <= 45;
    }

    @Override
    protected boolean isMirroredTargetSingleTag(int id) {
        return id >= 42 && id <= 45;
    }

    @Override
    protected String targetNameForSingleTag(int id) {
        return id <= 41 ? "BLUE AUDIENCE" : "BLUE SCORING";
    }

    @Override
    protected String controlsDescription() {
        return "BLUE TELEOP: shooter auto speed; RB feeds; hold gp2 A for nearby aim assist";
    }
}
