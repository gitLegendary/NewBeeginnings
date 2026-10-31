
        package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.action.mecanumdrive;
import org.firstinspires.ftc.teamcode.action.flywheelpid;

@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop extends OpMode {

    // Drivetrain
    private mecanumdrive drive = new mecanumdrive();

    // Flywheel
    private flywheelpid launcher = new flywheelpid();

    // Intake motors
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Flower servo
    private Servo flowerServo;

    // Servo toggle state
    private boolean flowerExtended = false;
    private boolean previousA = false;

    // Servo positions
    private static final double FLOWER_RETRACTED = 0.0;
    private static final double FLOWER_EXTENDED = 1.0;

    // Flywheel RPM adjustment
    private static final double RPM_INCREMENT = 100.0;

    @Override
    public void init() {

        // Initialize drivetrain
        drive.init(this);

        // Initialize flywheel
        launcher.init(this);

        // Initialize intakes
        intakeLeft = hardwareMap.get(
                DcMotor.class, "Intake Left");
        intakeRight = hardwareMap.get(
                DcMotor.class, "Intake Right");

        // Initialize Flower servo
        flowerServo = hardwareMap.get(
                Servo.class, "Flower Servo");

        // Set initial servo position
        flowerServo.setPosition(FLOWER_RETRACTED);
        flowerExtended = false;

        // Initialize intake motors
        intakeLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TeleOp Initialized");
        telemetry.addLine("Gamepad 1 A: Toggle Flower Servo");
        telemetry.addLine("Gamepad 2 Bumpers: Control Intakes");
        telemetry.update();
    }

    @Override
    public void loop() {

        // =========================================
        // FIELD-CENTRIC DRIVETRAIN - GAMEPAD 1
        // =========================================

        drive.setFieldCentricPower(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // FLOWER SERVO - GAMEPAD 2 A

        boolean currentA = gamepad2.a;

        if (currentA && !previousA) {
            flowerExtended = !flowerExtended;

            if (flowerExtended) {
                flowerServo.setPosition(FLOWER_EXTENDED);
            } else {
                flowerServo.setPosition(FLOWER_RETRACTED);
            }
        }

        previousA = currentA;

        previousA = currentA;

        // =========================================
        // FLYWHEEL - GAMEPAD 2
        // =========================================

        // Maintain flywheel control
        launcher.setMotorRPM(
                gamepad2.left_trigger,
                gamepad2.right_trigger,
                gamepad2.x,
                0
        );

        // Adjust target RPM with the D-pad
        if (gamepad2.dpad_up) {
            launcher.lowVelocity += RPM_INCREMENT;
        }

        if (gamepad2.dpad_down) {
            launcher.lowVelocity -= RPM_INCREMENT;
        }

        // =========================================
        // INTAKES - GAMEPAD 2
        // =========================================

        if (gamepad2.right_bumper
                && !gamepad2.left_bumper) {

            // Intake forward
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);

        } else if (gamepad2.left_bumper
                && !gamepad2.right_bumper) {

            // Intake reverse
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);

        } else {

            // Stop intakes
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }


        if (gamepad2.right_trigger_pressed){
            // Run the flywheel backward at 50 RPM
            launcher.setMotorRPM(-50.0f, 0.0f, false, 0);

        }

        // =========================================
        // TELEMETRY
        // =========================================

        telemetry.addData(
                "Flower Servo",
                flowerExtended ? "Extended" : "Retracted"
        );

        telemetry.addData(
                "Flywheel RPM",
                "%.0f",
                launcher.getRPM()
        );

        telemetry.addData(
                "Flywheel Target RPM",
                "%.0f",
                launcher.lowVelocity
        );

        telemetry.addData(
                "Intake",
                gamepad2.right_bumper
                        && !gamepad2.left_bumper
                        ? "Forward"
                        : gamepad2.left_bumper
                        && !gamepad2.right_bumper
                        ? "Reverse"
                        : "Stopped"
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        // Stop intakes when TeleOp ends
        if (intakeLeft != null) {
            intakeLeft.setPower(0);
        }

        if (intakeRight != null) {
            intakeRight.setPower(0);
        }

        // Stop flywheel when TeleOp ends
        if (launcher != null) {
            launcher.setMotorPower(0);
        }

        // Stop drivetrain
        if (drive != null) {
            drive.stop();
        }
    }
}
