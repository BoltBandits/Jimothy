package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Read-only diagnostic that identifies odometry pods connected to motor encoder ports. */
@TeleOp(name = "BioBuzz Odometry Identifier", group = "Diagnostics")
public class BioBuzzOdometryIdentifier extends OpMode {
    private static final String[] DEVICE_NAMES = {
            "intake_motor",
            "odo_right",
            "odo_center"
    };

    private static final String[] PORT_LABELS = {
            "Left parallel / Expansion encoder 0",
            "Right parallel / Expansion encoder 2",
            "Center perpendicular / Expansion encoder 3"
    };

    private final DcMotor[] devices = new DcMotor[DEVICE_NAMES.length];
    private final int[] baseline = new int[DEVICE_NAMES.length];
    private boolean previousDpadUp;

    @Override
    public void init() {
        for (int i = 0; i < DEVICE_NAMES.length; i++) {
            devices[i] = hardwareMap.get(DcMotor.class, DEVICE_NAMES[i]);
            devices[i].setPower(0);
            devices[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            baseline[i] = devices[i].getCurrentPosition();
        }
        telemetry.addLine("READ ONLY: no motors will be powered");
        telemetry.addLine("Spin one odometry pod by hand, then watch its delta");
    }

    @Override
    public void start() {
        captureBaseline();
    }

    @Override
    public void loop() {
        boolean reset = gamepad1.dpad_up || gamepad2.dpad_up;
        if (reset && !previousDpadUp) {
            captureBaseline();
        }
        previousDpadUp = reset;

        telemetry.addLine("D-pad UP = reset deltas");
        for (int i = 0; i < devices.length; i++) {
            int raw = devices[i].getCurrentPosition();
            telemetry.addData(PORT_LABELS[i], "%s  raw %d  delta %+d",
                    DEVICE_NAMES[i], raw, raw - baseline[i]);
        }
    }

    @Override
    public void stop() {
        for (DcMotor device : devices) {
            device.setPower(0);
        }
    }

    private void captureBaseline() {
        for (int i = 0; i < devices.length; i++) {
            baseline[i] = devices[i].getCurrentPosition();
        }
    }
}
