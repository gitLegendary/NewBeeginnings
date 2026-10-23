
package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "GoalPark1", group = "Autonomous")
public class GoalPark1 extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double STRAFE_POWER = 0.60;
    private static final double DRIVE_POWER = 0.60;

    // Movement durations in milliseconds
    private static final long AWAY_FROM_WALL_TIME_MS = 150;
    private static final long STRAFE_LEFT_TIME_MS = 1375;
    private static final long FORWARD_TIME_MS = 2250;

    @Override
    public void runOpMode() {

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

        telemetry.addLine("Blue Goal Park Auto");
        telemetry.addLine("Wait 3 seconds, move away from wall,");
        telemetry.addLine("strafe left, then drive forward.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // STEP 1: Wait 3 seconds
        stopDrive();
        sleep(3000);

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // STEP 2: Move slightly away from the wall.
        // Assumes robot-forward points away from the wall.
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                AWAY_FROM_WALL_TIME_MS
        );

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // STEP 3: Strafe left
        drive(
                -STRAFE_POWER, STRAFE_POWER,
                STRAFE_POWER, -STRAFE_POWER,
                STRAFE_LEFT_TIME_MS
        );

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // STEP 4: Drive forward into the blue parking zone
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                FORWARD_TIME_MS
        );

        stopDrive();

        telemetry.addLine("Blue parking route complete");
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