
package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "GoalDelayAuto1", group = "Autonomous")
public class GoalDelayAuto1 extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double STRAFE_POWER = 0.60;
    private static final double DRIVE_POWER = 0.60;

    // Adjustable movement durations
    private static final long INITIAL_FORWARD_TIME_MS = 150;
    private static final long STRAFE_RIGHT_TIME_MS = 1500;
    private static final long FINAL_FORWARD_TIME_MS = 500;

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

        // Existing motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        telemetry.addLine("GoalDelayAuto");
        telemetry.addLine("Wait 10 seconds");
        telemetry.addLine("Move forward");
        telemetry.addLine("Strafe right 30 inches");
        telemetry.addLine("Drive forward 12 inches");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // STEP 1: Wait 10 seconds
        stopDrive();
        sleep(10000);

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // STEP 2: Move forward briefly
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                INITIAL_FORWARD_TIME_MS
        );

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // STEP 3: Strafe right approximately 30 inches
        drive(
                STRAFE_POWER, -STRAFE_POWER,
                -STRAFE_POWER, STRAFE_POWER,
                STRAFE_RIGHT_TIME_MS
        );

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // STEP 4: Drive forward approximately 12 inches
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                FINAL_FORWARD_TIME_MS
        );

        // STEP 5: Stop
        stopDrive();

        telemetry.addLine("GoalDelayAuto complete");
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

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}
