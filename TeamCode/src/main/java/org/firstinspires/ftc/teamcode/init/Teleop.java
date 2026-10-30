
        package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.action.mecanumdrive;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {

    // =========================
    // FLYWHEEL CONTROLLER
    // =========================

    public static class flywheelpid {

        private DcMotorEx m1;

        private double encoderCPM = 28;
        private double gearRatio = 3.0 / 2.0;

        private double kV = 0.000171;
        private double kS = 0.09;
        private double kP = 0.0009;

        public double lowVelocity = 2000;
        public double highVelocity = 3500;

        private double targetRpm;

        private Telemetry telemetry;

        private boolean lastX = false;
        private boolean flywheelToggle = false;

        private boolean lastDpadUp = false;
        private boolean lastDpadDown = false;

        // Initialize flywheel
        public void init(@NonNull OpMode opMode) {

            telemetry = opMode.telemetry;

            m1 = opMode.hardwareMap.get(
                    DcMotorEx.class,
                    "Turret Launcher"
            );

            m1.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );

            m1.setPower(0);
        }

        // Set flywheel power
        public void setMotorPower(double power) {

            m1.setPower(
                    Range.clip(power, 0.0, 1.0)
            );
        }

        // Flywheel speed control
        public void setMotorRPM(
                float leftTrigger,
                float rightTrigger,
                boolean x,
                double distance
        ) {

            // Toggle distance mode with X
            if (x && !lastX) {
                flywheelToggle = !flywheelToggle;
            }

            if (leftTrigger > 0.5 && rightTrigger < 0.5) {

                targetRpm = lowVelocity;

            } else if (rightTrigger > 0.5 && leftTrigger < 0.5) {

                targetRpm = highVelocity;

            } else if (flywheelToggle) {

                targetRpm = Range.clip(
                        -0.0366411 * Math.pow(distance, 2)
                                + 21.04711 * distance
                                + 707.85114,
                        0,
                        4000
                );

            } else {

                targetRpm = 0;
                setMotorPower(0);
                lastX = x;
                return;
            }

            // Calculate speed error
            double error = targetRpm - getRPM();

            // Feedforward and feedback
            double ff = (kV * targetRpm) + kS;
            double fb = error * kP;

            double power = Range.clip(
                    ff + fb,
                    0.0,
                    1.0
            );

            setMotorPower(power);

            // Save button state
            lastX = x;
        }

        // Adjust low-speed target
        public void adjustLowVelocity(
                boolean dpadUp,
                boolean dpadDown
        ) {

            if (dpadUp && !lastDpadUp) {
                lowVelocity += 10;
            }

            if (dpadDown && !lastDpadDown) {
                lowVelocity -= 10;
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
        }

        // Read encoder velocity
        public double getTicksPerSec() {
            return m1.getVelocity();
        }

        // Calculate flywheel RPM
        public double getRPM() {
            return (
                    (getTicksPerSec() / encoderCPM) * 60.0
            ) / gearRatio;
        }

        // Flywheel telemetry
        public void telemetryOutput() {

            telemetry.addData(
                    "Flywheel Target RPM",
                    "%.0f",
                    targetRpm
            );

            telemetry.addData(
                    "Flywheel Actual RPM",
                    "%.0f",
                    getRPM()
            );

            telemetry.addData(
                    "Flywheel Motor Power",
                    "%.2f",
                    m1.getPower()
            );

            telemetry.addData(
                    "Flywheel Mode",
                    flywheelToggle ? "Distance" : "Manual"
            );
        }
    }
    private mecanumdrive drive = new mecanumdrive();
    private flywheelpid launcher = new flywheelpid();

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    @Override
    public void init() {
        drive.init(this);
        launcher.init(this);

        intakeLeft = hardwareMap.get(
                DcMotor.class, "Intake Left"
        );

        intakeRight = hardwareMap.get(
                DcMotor.class, "Intake Right"
        );

        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        telemetry.addLine("Field Centric Drive Ready");
        telemetry.addLine("Flywheel Initialized");
        telemetry.addLine("Both Intakes Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Drivetrain
        drive.setFieldCentricPower(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        drive.slowMode(gamepad1.left_trigger > 0.1);

        if (gamepad1.options) {
            drive.resetHeading();
        }

        // Flywheel
        launcher.setMotorRPM(
                gamepad2.left_trigger,
                gamepad2.right_trigger,
                gamepad2.x,
                0
        );

        launcher.adjustLowVelocity(
                gamepad2.dpad_up,
                gamepad2.dpad_down
        );

        // Two intakes
        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);

        } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);

        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

        // Telemetry
        drive.telemetryOutput();
        launcher.telemetryOutput();

        telemetry.addData("Left Intake Power", intakeLeft.getPower());
        telemetry.addData("Right Intake Power", intakeRight.getPower());

        telemetry.update();
    }
}


    // =========================
    // MAIN TELEOP VARIABLES
    // =========================

