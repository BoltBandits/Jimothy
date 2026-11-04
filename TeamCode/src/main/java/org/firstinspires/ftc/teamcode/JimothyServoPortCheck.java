package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;

/** Safely identifies CR servos by testing one physical Control Hub port at a time. */
@TeleOp(name = "Jimothy Servo Port Check", group = "Diagnostics")
public class JimothyServoPortCheck extends OpMode {
    private static final int FIRST_PORT = 0;
    private static final int LAST_PORT = 5;
    private static final double STOP_POSITION = 0.50;
    private static final double FORWARD_POSITION = 0.625;
    private static final double REVERSE_POSITION = 0.375;

    private ServoController servoController;
    private int selectedPort;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;

    @Override
    public void init() {
        servoController = hardwareMap.get(ServoController.class, "Control Hub");
        stopAllPorts();

        telemetry.addLine("REMOVE GAME PIECES and keep clear of every mechanism");
        telemetry.addLine("D-pad Left/Right selects physical servo port 0-5");
        telemetry.addLine("Hold A=one direction | Hold B=opposite direction");
        telemetry.addLine("Only the selected port runs; release to stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right && !previousDpadRight) {
            stopAllPorts();
            selectedPort = selectedPort == LAST_PORT ? FIRST_PORT : selectedPort + 1;
        }
        if (gamepad1.dpad_left && !previousDpadLeft) {
            stopAllPorts();
            selectedPort = selectedPort == FIRST_PORT ? LAST_PORT : selectedPort - 1;
        }
        previousDpadRight = gamepad1.dpad_right;
        previousDpadLeft = gamepad1.dpad_left;

        stopAllPorts();
        String command = "STOPPED";
        if (gamepad1.a) {
            servoController.setServoPosition(selectedPort, FORWARD_POSITION);
            command = "DIRECTION A";
        } else if (gamepad1.b) {
            servoController.setServoPosition(selectedPort, REVERSE_POSITION);
            command = "DIRECTION B";
        }

        telemetry.addData("SELECTED PHYSICAL PORT", selectedPort);
        telemetry.addData("Command", command);
        telemetry.addData("Test power", "25%%");
        telemetry.addLine("D-pad Left/Right = change port");
        telemetry.addLine("Hold A or B = spin selected port | release = stop");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllPorts();
    }

    private void stopAllPorts() {
        for (int port = FIRST_PORT; port <= LAST_PORT; port++) {
            servoController.setServoPosition(port, STOP_POSITION);
        }
    }
}
