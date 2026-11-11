package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Drives straight backward eight inches, stops, and requires no mechanisms or sensors. */
@Autonomous(name = "Jimothy Simplest Park", group = "Jimothy")
public class JimothySimplestPark extends LinearOpMode {
    private static final double DISTANCE_INCHES = 8.0;
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double DRIVE_POWER = -0.50;
    private static final double TIMEOUT_SECONDS = 12.0;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
        }

        int targetTicks = (int) Math.round(DISTANCE_INCHES * LEFT_TICKS_PER_INCH);
        telemetry.addLine("JIMOTHY SIMPLEST PARK READY");
        telemetry.addData("Move", "%.1f inches backward", DISTANCE_INCHES);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        int startTicks = frontLeft.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime();
        boolean reachedTarget;
        try {
            setDrivePower(DRIVE_POWER);
            while (opModeIsActive()
                    && Math.abs(frontLeft.getCurrentPosition() - startTicks) < targetTicks
                    && timer.seconds() < TIMEOUT_SECONDS) {
                int traveledTicks = Math.abs(frontLeft.getCurrentPosition() - startTicks);
                telemetry.addData("Step", "DRIVING BACKWARD");
                telemetry.addData("Distance", "%.2f / %.2f inches",
                        traveledTicks / LEFT_TICKS_PER_INCH, DISTANCE_INCHES);
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
        }
        reachedTarget = Math.abs(frontLeft.getCurrentPosition() - startTicks) >= targetTicks;

        while (opModeIsActive()) {
            stopDrive();
            telemetry.addData("Step", reachedTarget ? "PARKED - TARGET REACHED"
                    : "PARKED - 12 SECOND TIMEOUT");
            telemetry.addData("Distance", "%.2f inches",
                    Math.abs(frontLeft.getCurrentPosition() - startTicks)
                            / LEFT_TICKS_PER_INCH);
            telemetry.update();
            idle();
        }
    }

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopDrive() {
        setDrivePower(0.0);
    }
}
