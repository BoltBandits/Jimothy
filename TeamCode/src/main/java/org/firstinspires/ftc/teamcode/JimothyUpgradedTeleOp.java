package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Jimothy TeleOp with four selectable shooter speeds on gamepad 2's D-pad.
 * All other controls and behavior are inherited unchanged from Jimothy TeleOp.
 */
@TeleOp(name = "Jimothy Upgraded TeleOp", group = "Jimothy")
public class JimothyUpgradedTeleOp extends BioBuzzStarterbotTeleopV6 {
    public static final double NORMAL_SHOOTER_FRACTION = 0.45;
    public static final double DPAD_UP_SHOOTER_FRACTION = 0.48;
    public static final double DPAD_RIGHT_SHOOTER_FRACTION = 0.51;
    public static final double DPAD_DOWN_SHOOTER_FRACTION = 0.55;
    public static final double DPAD_LEFT_SHOOTER_FRACTION = 0.57;

    private String shooterMode = "NORMAL";

    @Override
    protected double normalShooterFraction() {
        return NORMAL_SHOOTER_FRACTION;
    }

    @Override
    protected void updateTurbo() {
        double selectedFraction = NORMAL_SHOOTER_FRACTION;
        shooterMode = "NORMAL";

        // This order also provides deterministic behavior if multiple directions are held.
        if (gamepad2.dpad_up) {
            selectedFraction = DPAD_UP_SHOOTER_FRACTION;
            shooterMode = "D-PAD UP";
        } else if (gamepad2.dpad_right) {
            selectedFraction = DPAD_RIGHT_SHOOTER_FRACTION;
            shooterMode = "D-PAD RIGHT";
        } else if (gamepad2.dpad_down) {
            selectedFraction = DPAD_DOWN_SHOOTER_FRACTION;
            shooterMode = "D-PAD DOWN";
        } else if (gamepad2.dpad_left) {
            selectedFraction = DPAD_LEFT_SHOOTER_FRACTION;
            shooterMode = "D-PAD LEFT";
        }

        // The base class treats an alternate-speed button as a direct shoot command.
        turbo = gamepad2.dpad_up || gamepad2.dpad_right
                || gamepad2.dpad_down || gamepad2.dpad_left;
        launcherTargetVelocity = selectedFraction * launcherMaxVelocity;
    }

    @Override
    protected String shooterSettingLabel() {
        return shooterMode;
    }

    @Override
    protected String shooterSettingsDescription() {
        return "RB 45%; D-pad up/right/down/left 48/51/55/57%";
    }

    @Override
    protected String controlsDescription() {
        return "UPGRADED: RB 45%; D-pad U/R/D/L shoots 48/51/55/57%";
    }
}
