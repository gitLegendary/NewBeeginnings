package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.flywheelpid;

@Autonomous(name = "GoalParkAuto1", group = "Autonomous")
public class GoalDelayAuto1 extends LinearOpMode {

    // Drive settings
    private static final double DRIVE_POWER = 0.30;
    private static final double STRAFE_POWER = 0.30;

    // Driving times
    private static final long INITIAL_FORWARD_TIME_MS = 800;
    private static final long STRAFE_LEFT_TIME_MS = 2000;
    private static final long FINAL_FORWARD_TIME_MS = 3000;

    // Initial delay while the flywheel spins up
    private static final double INITIAL_DELAY_SECONDS = 3.0;

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

        // ==========================================
        // INITIALIZE DRIVE MOTORS
        // ==========================================

        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");

        // ==========================================
        // INITIALIZE INTAKE MOTORS
        // ==========================================

        intakeLeft = hardwareMap.get(DcMotor.class, "Intake Left");
        intakeRight = hardwareMap.get(DcMotor.class, "Intake Right");

        // ==========================================
        // INITIALIZE FLYWHEEL
        // ==========================================

        launcher = new flywheelpid();
        launcher.init(this);
        launcher.lowVelocity = TARGET_RPM;

        // ==========================================
        // CONFIGURE DRIVE MOTOR DIRECTIONS
        // ==========================================

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // ==========================================
        // CONFIGURE BRAKING
        // ==========================================

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intakeLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        intakeRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // ==========================================
        // READY TO START
        // ==========================================

        telemetry.addLine("BlueGoalParkAuto initialized");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addLine("Waiting for START");
        telemetry.update();

        // Wait until the driver presses START
        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        // ==========================================
        // INITIAL THREE-SECOND DELAY
        // Spin up the flywheel while stationary
        // ==========================================

        ElapsedTime delayTimer = new ElapsedTime();

        while (opModeIsActive()
                && delayTimer.seconds() < INITIAL_DELAY_SECONDS) {

            // Command the flywheel to spin up
            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            // Keep the robot and intakes stationary
            stopDrive();
            intakeLeft.setPower(0);
            intakeRight.setPower(0);

            telemetry.addData(
                    "Status",
                    "Initial delay: spinning up"
            );
            telemetry.addData(
                    "Elapsed",
                    "%.1f seconds",
                    delayTimer.seconds()
            );
            telemetry.addData(
                    "Flywheel RPM",
                    "%.0f",
                    launcher.getRPM()
            );
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

            stopDrive();
            intakeLeft.setPower(0);
            intakeRight.setPower(0);

            telemetry.addData(
                    "Status",
                    "Waiting for flywheel RPM"
            );
            telemetry.addData(
                    "Flywheel RPM",
                    "%.0f",
                    launcher.getRPM()
            );
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData(
                    "Spin-up time",
                    "%.1f seconds",
                    spinupTimer.seconds()
            );
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
        // SHOOT FOR THREE SECONDS
        // ==========================================

        ElapsedTime shootingTimer = new ElapsedTime();

        while (opModeIsActive()
                && shootingTimer.milliseconds() < SHOOTING_TIME_MS) {

            // Maintain flywheel command
            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            // Feed game pieces into the launcher
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);

            // Keep the drive motors stopped
            stopDrive();

            telemetry.addData("Status", "Shooting");
            telemetry.addData(
                    "Flywheel RPM",
                    "%.0f",
                    launcher.getRPM()
            );
            telemetry.addData(
                    "Shooting time",
                    "%.1f seconds",
                    shootingTimer.seconds()
            );
            telemetry.update();

            idle();
        }

        // ==========================================
        // STOP SHOOTING MECHANISMS
        // ==========================================

        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        launcher.setMotorPower(0);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // ==========================================
        // MOVE FORWARD AWAY FROM THE WALL
        // ==========================================

        driveForward(
                DRIVE_POWER,
                INITIAL_FORWARD_TIME_MS
        );

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // ==========================================
        // STRAFE LEFT
        // ==========================================

        strafeLeft(
                STRAFE_POWER,
                STRAFE_LEFT_TIME_MS
        );

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // ==========================================
        // FINAL FORWARD MOVEMENT
        // ==========================================

        driveForward(
                DRIVE_POWER,
                FINAL_FORWARD_TIME_MS
        );

        // ==========================================
        // END AUTONOMOUS
        // ==========================================

        stopAll();

        telemetry.addLine("BlueGoalParkAuto complete");
        telemetry.update();
    }

    // ==========================================
    // DRIVE FORWARD
    // ==========================================

    private void driveForward(
            double power,
            long durationMs
    ) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.milliseconds() < durationMs) {

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

            idle();
        }

        stopDrive();
    }

    // ==========================================
    // STRAFE LEFT
    // ==========================================

    private void strafeLeft(
            double power,
            long durationMs
    ) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.milliseconds() < durationMs) {

            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);

            idle();
        }

        stopDrive();
    }

    // ==========================================
    // STOP DRIVE MOTORS
    // ==========================================

    private void stopDrive() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
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