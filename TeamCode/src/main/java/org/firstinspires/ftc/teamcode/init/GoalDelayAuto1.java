
package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.flywheelpid;

@Autonomous(name = "GoalDelayAuto1", group = "Autonomous")
public class GoalDelayAuto1 extends LinearOpMode {

    // Drive motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    // Intake motors
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Launch servo
    private Servo launchServo;

    // Flywheel controller
    private flywheelpid launcher;

    // Drive settings
    private static final double DRIVE_POWER = 0.30;
    private static final double STRAFE_POWER = 0.30;

    // Movement times
    private static final long STRAFE_LEFT_TIME_MS = 2000;
    private static final long FINAL_FORWARD_TIME_MS = 1000;

    // Shooting settings
    private static final double TARGET_RPM = 2750;
    private static final double RPM_TOLERANCE = 100;

    private static final double INITIAL_DELAY_SECONDS = 3.0;
    private static final long FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;
    private static final long SHOOTING_TIME_MS = 3000;

    private static final long SERVO_EXTEND_TIME_MS = 500;
    private static final long SERVO_RETRACT_TIME_MS = 500;

    private static final double SERVO_RETRACTED = 0.0;
    private static final double SERVO_EXTENDED = 1.0;

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

        // Initialize servo
        launchServo = hardwareMap.get(Servo.class, "Launch Servo");

        // Initialize flywheel
        launcher = new flywheelpid();
        launcher.init(this);
        launcher.lowVelocity = TARGET_RPM;

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake when motors stop
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initial states
        stopAll();
        launchServo.setPosition(SERVO_RETRACTED);

        telemetry.addLine("GoalDelayAuto1 initialized");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        // --------------------------------
        // SPIN UP FLYWHEEL DURING THE DELAY
        // --------------------------------

        ElapsedTime delayTimer = new ElapsedTime();

        while (opModeIsActive()
                && delayTimer.seconds() < INITIAL_DELAY_SECONDS) {

            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            telemetry.addData("Stage", "Initial delay");
            telemetry.addData("Flywheel RPM", launcher.getRPM());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // --------------------------------
        // WAIT FOR FLYWHEEL TO REACH TARGET
        // --------------------------------

        ElapsedTime spinupTimer = new ElapsedTime();

        while (opModeIsActive()
                && launcher.getRPM() < TARGET_RPM - RPM_TOLERANCE
                && spinupTimer.milliseconds() < FLYWHEEL_SPINUP_TIMEOUT_MS) {

            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            telemetry.addData("Stage", "Spinning up");
            telemetry.addData("Flywheel RPM", launcher.getRPM());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // --------------------------------
        // SHOOT
        // --------------------------------

        launcher.setMotorRPM(1.0f, 0.0f, false, 0);

        // Extend launch servo
        launchServo.setPosition(SERVO_EXTENDED);
        runFlywheelFor(SERVO_EXTEND_TIME_MS);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // Run intakes while shooting
        intakeLeft.setPower(-1.0);
        intakeRight.setPower(-1.0);

        runFlywheelFor(SHOOTING_TIME_MS);

        // Stop intakes
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        runFlywheelFor(SERVO_RETRACT_TIME_MS);

        // Retract launch servo
        launchServo.setPosition(SERVO_RETRACTED);

        // Stop flywheel after shooting
        launcher.setMotorPower(0);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // --------------------------------
        // STRAFE LEFT FOR 2 SECONDS
        // --------------------------------

        strafeLeft(STRAFE_POWER, STRAFE_LEFT_TIME_MS);

        if (!opModeIsActive()) {
            stopAll();
            return;
        }

        // --------------------------------
        // DRIVE FORWARD FOR 1 SECOND
        // --------------------------------

        driveForward(DRIVE_POWER, FINAL_FORWARD_TIME_MS);

        // --------------------------------
        // END AUTONOMOUS
        // --------------------------------

        stopAll();

        telemetry.addLine("GoalDelayAuto1 complete");
        telemetry.update();
    }

    // --------------------------------
    // STRAFE LEFT
    // --------------------------------

    private void strafeLeft(double power, long durationMs) {

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

    // --------------------------------
    // DRIVE FORWARD
    // --------------------------------

    private void driveForward(double power, long durationMs) {

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

    // --------------------------------
    // KEEP FLYWHEEL CONTROL ACTIVE
    // --------------------------------

    private void runFlywheelFor(long durationMs) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.milliseconds() < durationMs) {

            launcher.setMotorRPM(1.0f, 0.0f, false, 0);

            telemetry.addData("Flywheel RPM", launcher.getRPM());
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.update();

            idle();
        }
    }

    // --------------------------------
    // STOP DRIVE MOTORS
    // --------------------------------

    private void stopDrive() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // --------------------------------
    // STOP EVERYTHING
    // --------------------------------

    private void stopAll() {

        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);

        if (intakeLeft != null) intakeLeft.setPower(0);
        if (intakeRight != null) intakeRight.setPower(0);

        if (launcher != null) launcher.setMotorPower(0);
    }
}
