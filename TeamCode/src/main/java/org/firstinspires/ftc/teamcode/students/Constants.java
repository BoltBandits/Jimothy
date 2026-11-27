package org.firstinspires.ftc.teamcode.students;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    /*
     * THESE ARE ONLY FOR USE IN RUNNING THE TUNING OPMODES IN VIRTUAL_ROBOT, WHICH
     * ARE DISABLED BY DEFAULT. THERE SHOULD BE NO NEED TO RETUNE.  THEY ARE
     * NOT NEEDED FOR A REAL ROBOT!! THEY ALSO ARE NOT NEEDED FOR YOUR CUSTOM OPMODES
     * OR THE TESTING OPMODES IN VIRTUAL_ROBOT.
     */
    public static double tuningHeadingLinear = 0.0;
    public static double tuningHeadingQuadratic = 0.0;
    public static double tuningHeading = 0.0;
    public static double tuningForwardLinear = 0.0;
    public static double tuningForwardQuadratic = 0.0;
    public static double tuningStrafeLinear = 0.0;
    public static double tuningStrafeQuadratic = 0.0;

    /*
     * THE TUNING VALUES BELOW ARE FOR THE MecDynamicBot CONFIGURATION OF
     * VIRTUAL_ROBOT. THAT IS THE RECOMMENDED CONFIGURATION FOR USING
     * PEDROPATHING.
     *
     * THERE SHOULD BE NO NEED TO RETUNE, AS THESE HAVE BEEN CREATED BY RUNNING
     * THE TUNING OPMODES OF THE PEDRO QUICKSTART (ADAPTED AS NEEDED FOR THE
     * VIRTUAL_ROBOT PROJECT).
     */

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front_drive");
                c.backLeftName.set("left_back_drive");
                c.frontRightName.set("right_front_drive");
                c.backRightName.set("right_back_drive");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );
    public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(
            c -> {
                c.leftEncoderName.set("left_front_drive");
                c.rightEncoderName.set("odo_right");
                c.strafeEncoderName.set("odo_center");

                double forwardTicksPerInch = (503.47 + 504.53) / 2.0; // 504.00

                c.leftPodY.set(-12.10 / 2.0);    // -6.05
                c.rightPodY.set(12.10 / 2.0);  // 6.05
                c.strafePodX.set(-6.56);

                c.forwardTicksToInches.set(1.0 / forwardTicksPerInch);
                c.strafeTicksToInches.set(1.0 / 510.30);
                c.turnTicksToRadians.set(1.0 / forwardTicksPerInch);

                c.leftEncoderDirection.set(Encoder.REVERSE);
                c.rightEncoderDirection.set(Encoder.FORWARD);
                c.strafeEncoderDirection.set(Encoder.REVERSE);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29);
                Controller secondaryTranslationalForward = Controller.proportional(0.11);
                Controller primaryTranslationalLateral = Controller.proportional(0.29);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11);
//                c.forwardTranslational.set(secondaryTranslationalForward);
//                c.strafeTranslational.set(secondaryTranslationalLateral);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
                c.coast.set(Controller.proportionalFeedforward(0.02));
                c.brake.set(Controller.proportionalFeedforward(0.017));
                c.headingFeedback.set(Controller.proportional(3.14));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045, 0.0022));
                c.linearBrakeCoefficients.set(Matrix.diag(0.11, 0.14));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017, 1.6e-4));
                c.maxAchievableForwardVelocity.set(49.9);
                c.maxAchievableStrafeVelocity.set(49.9);
                c.naturalForwardDeceleration.set(24.4);
                c.naturalStrafeDeceleration.set(24.4);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new ThreeWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}

