package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.flywheelpid;

@Autonomous(name = "GoalDelayAuto1", group = "Autonomous")
public class GoalDelayAuto1 extends LinearOpMode {

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
    private static final double DRIVE_POWER = 0.60;
    private static final double STRAFE_POWER = 0.60;

    // Movement durations
    private static final long INITIAL_FORWARD_TIME_MS = 300;
    private static final long STRAFE_RIGHT_TIME_MS = 2500;
    private static final long FINAL_FORWARD_TIME_MS = 1000;

    // Shooting settings
    private static final double TARGET_RPM = 2000;
    private static final double RPM_TOLERANCE = 100;
    private static final long SHOOTING_TIME_MS = 3000;
    private static final long FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;

    @Override
    public void runOpMode() {

        // Initialize drivetrain
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");

        // Initialize intakes
        intakeLeft = hardwareMap.get(DcMotor.class, "Intake Left");
        intakeRight = hardwareMap.get(DcMotor.class, "Intake Right");

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
        DcMotor[] driveMotors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        // Configure intakes
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAll();

        telemetry.addLine("GoalDelayAuto1 Ready");
        telemetry.addLine("Sequence: Wait -> Shoot -> Move");
        telemetry.addData("Target Flywheel RPM", TARGET_RPM);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        // STEP 1: Wait five seconds without moving
        stopAll();
        sleep(5000);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // STEP 2: Shoot while the robot remains stationary
        boolean shootingSucceeded = shootWhileStationary();

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // Do not move if the flywheel fails to reach its target speed
        if (!shootingSucceeded) {
            stopAll();
            telemetry.addLine("Shooting failed. Autonomous stopped.");
            telemetry.update();
            return;
        }

        // STEP 3: Drive forward
        drive(
                DRIVE_POWER,
                DRIVE_POWER,
                DRIVE_POWER,
                DRIVE_POWER,
                INITIAL_FORWARD_TIME_MS
        );

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // STEP 4: Strafe right
        drive(
                STRAFE_POWER,
                -STRAFE_POWER,
                -STRAFE_POWER,
                STRAFE_POWER,
                STRAFE_RIGHT_TIME_MS
        );

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // STEP 5: Final forward movement
        drive(
                DRIVE_POWER,
                DRIVE_POWER,
                DRIVE_POWER,
                DRIVE_POWER,
                FINAL_FORWARD_TIME_MS
        );

        // STEP 6: Stop everything
        stopAll();

        telemetry.addLine("GoalDelayAuto1 Complete");
        telemetry.update();
    }

    /**
     * Spins up the flywheel and shoots without moving the robot.
     */
    private boolean shootWhileStationary() {

        stopDrive();
        stopIntakes();

        // Spin up the flywheel
        timer.reset();

        while (opModeIsActive()
                && timer.milliseconds() < FLYWHEEL_SPINUP_TIMEOUT_MS) {

            updateFlywheel();

            double actualRPM = launcher.getRPM();

            telemetry.addData("Status", "Spinning up");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Actual RPM", "%.0f", actualRPM);
            telemetry.update();

            if (actualRPM >= TARGET_RPM - RPM_TOLERANCE) {
                break;
            }

            idle();
        }

        if (!opModeIsActive()) {
            stopAll();
            return false;
        }

        // Confirm that the flywheel reached the required speed
        if (launcher.getRPM() < TARGET_RPM - RPM_TOLERANCE) {
            stopAll();
            return false;
        }

        // Shoot for three seconds while remaining stationary
        intakeLeft.setPower(1.0);
        intakeRight.setPower(1.0);

        timer.reset();

        while (opModeIsActive()
                && timer.milliseconds() < SHOOTING_TIME_MS) {

            stopDrive();
            updateFlywheel();

            telemetry.addData("Status", "Shooting while stationary");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData(
                    "Actual RPM",
                    "%.0f",
                    launcher.getRPM()
            );
            telemetry.update();

            idle();
        }

        // Stop the shooter and intakes before moving
        stopAll();

        return opModeIsActive();
    }

    /**
     * Maintains the flywheel target speed.
     */
    private void updateFlywheel() {
        launcher.setMotorRPM(1.0f, 0.0f, false, 0);

        telemetry.addData("Flywheel Target RPM", TARGET_RPM);
        telemetry.addData(
                "Flywheel Actual RPM",
                "%.0f",
                launcher.getRPM()
        );
    }

    /**
     * Drives for a specified duration.
     */
    private void drive(
            double fl,
            double fr,
            double bl,
            double br,
            long durationMs
    ) {
        stopDrive();
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