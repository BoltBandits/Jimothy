package org.firstinspires.ftc.teamcode;

/** FTC hardware-map names for devices and encoder inputs. */
public enum RobotHookups {
    // Matches the existing Jimothy drive calibration OpMode.
    JIMOTHY_LEGACY(Names.ODO_RIGHT, Names.FRONT_LEFT, Names.ODO_CENTER),
    // Control Hub encoder 0, Expansion Hub encoders 2 and 3.
    BIG_J_CHART(Names.FRONT_LEFT, Names.FEEDER, Names.ODO_CENTER);

    public final String rightOdometry;
    public final String leftOdometry;
    public final String strafeOdometry;

    RobotHookups(String rightOdometry, String leftOdometry, String strafeOdometry) {
        this.rightOdometry = rightOdometry;
        this.leftOdometry = leftOdometry;
        this.strafeOdometry = strafeOdometry;
    }

    public static final class Names {
        public static final String FRONT_LEFT = "left_front_drive";
        public static final String FRONT_RIGHT = "right_front_drive";
        public static final String BACK_LEFT = "left_back_drive";
        public static final String BACK_RIGHT = "right_back_drive";
        public static final String SWEEPER = "intake_motor";
        public static final String SHOOTER = "shooter_motor";
        public static final String FEEDER = "feeder_motor";
        public static final String ODO_RIGHT = "odo_right";
        public static final String ODO_CENTER = "odo_center";
        public static final String RIGHT_FLOWER_SWEEPER = "right_intake_servo";
        public static final String LEFT_FLOWER_SWEEPER = "left_intake_servo";
        public static final String WINDMILL = "windmill_servo";
        public static final String SHOOTER_TRIGGER = "shooter_trigger";
        public static final String TURRET = "turret";
        public static final String CONTROL_HUB_LIGHT = "control_hub_light";
        public static final String EXPANSION_HUB_LIGHT = "expansion_hub_light";
        public static final String LIMELIGHT = "limelight";
        public static final String SHOOTER_CAMERA = "Webcam 1";

        private Names() {}
    }
}
