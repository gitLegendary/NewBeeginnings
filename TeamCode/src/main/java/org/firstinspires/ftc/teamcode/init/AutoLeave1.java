
package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Leave 1", group = "Autonomous")
public class AutoLeave1 extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private final ElapsedTime timer = new ElapsedTime();

    // Adjustable drive settings
    private static final double DRIVE_POWER = 0.60;
    private static final double STRAFE_POWER = 0.60;

    // Initial estimates; tune on the field
    private static final long FORWARD_TIME_MS = 225;
    private static final long STRAFE_TIME_MS = 325;

    @Override
    public void runOpMode() {

        // Initialize motors
        frontRight = hardwareMap.get(
                DcMotor.class, "Front Right");
        frontLeft = hardwareMap.get(
                DcMotor.class, "Front Left");
        backRight = hardwareMap.get(
                DcMotor.class, "Back Right");
        backLeft = hardwareMap.get(
                DcMotor.class, "Back Left");

        // Your existing motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure drive motors
        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        telemetry.addLine("Drive By Time Auto");
        telemetry.addLine("Forward 6 inches");
        telemetry.addLine("Strafe right 6 inches");
        telemetry.addLine("Park");
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // STEP 1: Drive forward approximately 6 inches
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                FORWARD_TIME_MS
        );

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // Brief pause between movements
        pause(150);

        // STEP 2: Strafe right approximately 6 inches
        // Motor order: front left, front right,
        //              back left, back right
        drive(
                STRAFE_POWER, -STRAFE_POWER,
                -STRAFE_POWER, STRAFE_POWER,
                STRAFE_TIME_MS
        );

        // STEP 3: Stop in the parking zone
        stopDrive();

        telemetry.addLine("Parking movement complete");
        telemetry.update();
    }

    private void drive(
            double fl,
            double fr,
            double bl,
            double br,
            long durationMs) {

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        timer.reset();

        while (opModeIsActive()
                && timer.milliseconds() < durationMs) {
            idle();
        }

        stopDrive();
    }

    private void pause(long durationMs) {
        stopDrive();
        timer.reset();

        while (opModeIsActive()
                && timer.milliseconds() < durationMs) {
            idle();
        }
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}