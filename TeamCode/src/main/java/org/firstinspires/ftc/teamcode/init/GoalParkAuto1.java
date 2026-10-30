
        package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.flywheelpid;

@Autonomous(name = " GoalParkAuto1", group = "Autonomous")
public class GoalParkAuto1 extends LinearOpMode {

    // Drivetrain motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    // Intake motors
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Flywheel controller
    private flywheelpid launcher;

    private final ElapsedTime timer = new ElapsedTime();

    // Drive powers
    private static final double STRAFE_POWER = 0.30;
    private static final double DRIVE_POWER = 0.30;

    // Movement durations in milliseconds
    private static final long AWAY_FROM_WALL_TIME_MS = 300;
    private static final long STRAFE_LEFT_TIME_MS = 2000;
    private static final long FORWARD_TIME_MS = 3000;

    // Shooting settings
    private static final double TARGET_RPM = 2000;
    private static final double RPM_TOLERANCE = 100;

    private static final long SHOOTING_TIME_MS = 3000;
    private static final long FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;

    @Override
    public void runOpMode() {

        // Initialize drivetrain motors
        frontRight = hardwareMap.get(
                DcMotor.class, "Front Right");
        frontLeft = hardwareMap.get(
                DcMotor.class, "Front Left");
        backRight = hardwareMap.get(
                DcMotor.class, "Back Right");
        backLeft = hardwareMap.get(
                DcMotor.class, "Back Left");

        // Initialize intake motors
        intakeLeft = hardwareMap.get(
                DcMotor.class, "Intake Left");
        intakeRight = hardwareMap.get(
                DcMotor.class, "Intake Right");

        // Initialize flywheel
        launcher = new flywheelpid();
        launcher.init(this);
        launcher.lowVelocity = TARGET_RPM;

        // Configure drivetrain directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure drivetrain motors
        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        // Configure intakes
        intakeLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        stopIntakes();

        telemetry.addLine("Blue Goal Park Auto");
        telemetry.addLine("Wait 3 seconds, move, strafe, shoot, park.");
        telemetry.addData("Flywheel Target RPM", TARGET_RPM);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        // STEP 1: Wait 3 seconds
        stopAll();
        sleep(3000);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // STEP 2: Move away from the wall
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                AWAY_FROM_WALL_TIME_MS
        );

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // STEP 3: Strafe left
        drive(
                -STRAFE_POWER, STRAFE_POWER,
                STRAFE_POWER, -STRAFE_POWER,
                STRAFE_LEFT_TIME_MS
        );

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // STEP 4: Stop and shoot while stationary
        boolean shootingSucceeded =
                shootWhileStationary();

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // If the flywheel did not reach the required speed,
        // stop the autonomous rather than continuing to park.
        if (!shootingSucceeded) {
            stopAll();
            telemetry.addLine(
                    "Autonomous stopped: shooting was unsuccessful.");
            telemetry.update();
            return;
        }

        // STEP 5: Drive forward into the blue parking zone
        drive(
                DRIVE_POWER, DRIVE_POWER,
                DRIVE_POWER, DRIVE_POWER,
                FORWARD_TIME_MS
        );

        // STEP 6: Stop everything
        stopAll();

        telemetry.addLine("Blue parking route complete");
        telemetry.update();
    }

    /**
     * Runs the flywheel and intakes while the drivetrain remains stopped.
     * Returns true if the flywheel reaches the required RPM and the
     * shooting period completes.
     */
    private boolean shootWhileStationary() {

        stopDrive();

        // Start both intakes.
        intakeLeft.setPower(1.0);
        intakeRight.setPower(1.0);

        // STEP A: Spin up the flywheel.
        timer.reset();

        while (opModeIsActive()
                && timer.milliseconds()
                < FLYWHEEL_SPINUP_TIMEOUT_MS) {

            stopDrive();
            updateFlywheel();

            double actualRPM = launcher.getRPM();

            telemetry.addData("Status", "Spinning up flywheel");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Actual RPM", "%.0f", actualRPM);
            telemetry.update();

            // Start shooting only when the flywheel reaches
            // at least 1900 RPM.
            if (actualRPM >= TARGET_RPM - RPM_TOLERANCE) {
                break;
            }

            idle();
        }

        // Stop if the OpMode was interrupted.
        if (!opModeIsActive()) {
            stopAll();
            return false;
        }

        // Abort shooting if the flywheel failed to reach speed.
        if (launcher.getRPM() < TARGET_RPM - RPM_TOLERANCE) {

            telemetry.addLine(
                    "Flywheel did not reach required RPM.");
            telemetry.addLine("Shooting aborted.");
            telemetry.update();

            stopAll();
            return false;
        }

        // STEP B: Shoot for the full 3 seconds after reaching speed.
        timer.reset();

        while (opModeIsActive()
                && timer.milliseconds() < SHOOTING_TIME_MS) {

            // Keep the robot stationary.
            stopDrive();

            // Maintain flywheel speed and intake operation.
            updateFlywheel();

            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);

            telemetry.addData("Status", "Shooting");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData(
                    "Actual RPM", "%.0f", launcher.getRPM());
            telemetry.update();

            idle();
        }

        // Stop the shooter and intakes before moving.
        stopAll();

        return opModeIsActive();
    }

    /**
     * Updates the flywheel controller using the lowVelocity target.
     */
    private void updateFlywheel() {

        launcher.setMotorRPM(1.0f, 0.0f, false, 0);

        telemetry.addData(
                "Flywheel Target RPM", launcher.lowVelocity);
        telemetry.addData(
                "Flywheel Actual RPM", "%.0f", launcher.getRPM());
    }

    /**
     * Drives the robot for a specified duration.
     * The flywheel and intakes remain off during driving.
     */
    private void drive(
            double fl,
            double fr,
            double bl,
            double br,
            long durationMs) {

        // Ensure the shooter is stopped before driving.
        stopIntakes();

        if (launcher != null) {
            launcher.setMotorPower(0);
        }

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

    private void stopIntakes() {
        if (intakeLeft != null) intakeLeft.setPower(0);
        if (intakeRight != null) intakeRight.setPower(0);
    }

    private void stopAll() {
        stopDrive();
        stopIntakes();

        if (launcher != null) {
            launcher.setMotorPower(0);
        }
    }
}
