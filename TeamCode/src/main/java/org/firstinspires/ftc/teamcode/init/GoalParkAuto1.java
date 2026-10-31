package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.flywheelpid;

@Autonomous(name = "GoalDelayAuto1", group = "Autonomous")
public class GoalParkAuto1 extends LinearOpMode {

    // Drive settings
    private static final double DRIVE_POWER = 0.60;
    private static final double STRAFE_POWER = 0.60;

    // Driving times
    private static final long INITIAL_FORWARD_TIME_MS = 1000;
    private static final long STRAFE_RIGHT_TIME_MS = 1000;
    private static final long FINAL_FORWARD_TIME_MS = 1000;

    // Initial delay
    private static final double INITIAL_DELAY_SECONDS = 5.0;

    // Flywheel settings
    private static final double TARGET_RPM = 2750;
    private static final double RPM_TOLERANCE = 100;
    private static final long SHOOTING_TIME_MS = 3000;
    private static final long FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;

    // Drive motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    // Intake motors
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Flywheel controller
    private flywheelpid launcher;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");

        // Initialize intake motors
        intakeLeft = hardwareMap.get(DcMotor.class, "Intake Left");
        intakeRight = hardwareMap.get(DcMotor.class, "Intake Right");

        // Initialize flywheel
        launcher = new flywheelpid();
        launcher.init(this);
        launcher.lowVelocity = TARGET_RPM;

        // Configure drive motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure braking
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("GoalDelayAuto1 initialized");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        // ==========================================
        // INITIAL FIVE-SECOND DELAY
        // Spin up the flywheel while stationary
        // ==========================================

        ElapsedTime delayTimer = new ElapsedTime();

        while (opModeIsActive()
                && delayTimer.seconds() < INITIAL_DELAY_SECONDS) {

            // Keep commanding the flywheel during the delay
            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            // Keep the robot and intakes stationary
            stopDrive();
            intakeLeft.setPower(0);
            intakeRight.setPower(0);

            telemetry.addData("Status", "Initial delay: spinning up");
            telemetry.addData("Elapsed", "%.1f seconds",
                    delayTimer.seconds());
            telemetry.addData("Flywheel RPM", "%.0f",
                    launcher.getRPM());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // ==========================================
        // CONTINUE SPIN-UP IF NEEDED
        // ==========================================

        ElapsedTime spinupTimer = new ElapsedTime();

        while (opModeIsActive()
                && launcher.getRPM() < TARGET_RPM - RPM_TOLERANCE
                && spinupTimer.milliseconds()
                < FLYWHEEL_SPINUP_TIMEOUT_MS) {

            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            telemetry.addData("Status", "Waiting for target RPM");
            telemetry.addData("Flywheel RPM", "%.0f",
                    launcher.getRPM());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // Warn if the flywheel did not reach its target,
        // but allow the autonomous to continue.
        if (launcher.getRPM() < TARGET_RPM - RPM_TOLERANCE) {

            telemetry.addLine(
                    "WARNING: Flywheel did not reach target RPM."
            );
            telemetry.addData(
                    "Actual RPM",
                    "%.0f",
                    launcher.getRPM()
            );
            telemetry.addLine(
                    "Continuing autonomous."
            );
            telemetry.update();
        }

        // ==========================================
        // SHOOT WHILE STATIONARY
        // ==========================================

        ElapsedTime shootingTimer = new ElapsedTime();

        while (opModeIsActive()
                && shootingTimer.milliseconds() < SHOOTING_TIME_MS) {

            // Maintain flywheel speed
            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            // Reversed intake direction
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);

            telemetry.addData("Status", "Shooting");
            telemetry.addData("Flywheel RPM", "%.0f",
                    launcher.getRPM());
            telemetry.update();

            idle();
        }

        // Stop intake and flywheel before driving
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        launcher.setMotorPower(0);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // ==========================================
        // AUTONOMOUS DRIVING SEQUENCE
        // ==========================================

        strafeLeft(STRAFE_POWER, 1000);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        driveBackward(DRIVE_POWER, 1150);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        strafeLeft(STRAFE_POWER, 2500);
        driveBackward(0.30, 2000);
        driveForward(0.3, 250);
        stopAll();

        telemetry.addLine("GoalDelayAuto1 complete");
        telemetry.update();
    }

    private void driveBackward(double power, long durationMs) {

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        sleep(durationMs);

        stopDrive();
    }

    // ==========================================
    // DRIVE FORWARD
    // ==========================================

    private void driveForward(double power, long durationMs) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        sleep(durationMs);

        stopDrive();
    }

    // ==========================================
    // STRAFE RIGHT
    // ==========================================

    private void strafeRight(double power, long durationMs) {

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        sleep(durationMs);

        stopDrive();
    }
    private void strafeLeft(double power, long durationMs) {

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        sleep(durationMs);

        stopDrive();
    }
    // ==========================================
    // STOP DRIVE MOTORS
    // ==========================================

    private void stopDrive() {

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    // ==========================================
    // STOP ALL MOTORS
    // ==========================================

    private void stopAll() {

        stopDrive();

        if (intakeLeft != null) {
            intakeLeft.setPower(0);
        }

        if (intakeRight != null) {
            intakeRight.setPower(0);
        }

        if (launcher != null) {
            launcher.setMotorPower(0);
        }
    }
}
