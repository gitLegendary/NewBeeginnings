package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.action.mecanumdrive;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {

    public static class flywheelpid {
        private DcMotorEx m1;
        private double encoderCPM = 28;
        private double gearRatio = 3 / 2;
        private double kV = 0.000171, kS = 0.09, kP = 0.0009;

        public double lowVelocity = 2500;
        public double highVelocity = 3500;

        private double targetRpm;
        Telemetry telemetry;
        boolean lastX = false;
        boolean flywheelToggle = false;


        private boolean lastDpadUp = false;
        private boolean lastDpadDown = false;

        public void init(@NonNull OpMode opMode) {
            telemetry = opMode.telemetry;
            HardwareMap hardwareMap = opMode.hardwareMap;

            m1 = hardwareMap.get(DcMotorEx.class, "Turret Launcher");
            m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        public void setMotorPower(double power) {
            m1.setPower(power);
        }

        public void setMotorRPM(float leftTrigger, float rightTrigger, boolean x, double distance) {

            // Toggle flywheel mode when X is newly pressed
            if (x && !lastX) {
                flywheelToggle = !flywheelToggle;
            }

            if (leftTrigger > 0.5 && rightTrigger < 0.5) {
                targetRpm = lowVelocity;
                double error = targetRpm - getRPM();
                double ff = (kV * targetRpm) + kS;
                double fb = error * kP;
                double power = ff + fb;
                setMotorPower(power);

            } else if (rightTrigger > 0.5 && leftTrigger < 0.5) {
                targetRpm = highVelocity;
                double error = targetRpm - getRPM();
                double ff = (kV * targetRpm) + kS;
                double fb = error * kP;
                double power = ff + fb;
                setMotorPower(power);

            } else if (flywheelToggle) {
                targetRpm = Range.clip(-0.0366411 * Math.pow(distance, 2) + 21.04711 * distance + 707.85114, 0, 4000);
                double error = targetRpm - getRPM();
                double ff = (kV * targetRpm) + kS;
                double fb = error * kP;
                double power = ff + fb;
                setMotorPower(power);

            } else {

                setMotorPower(0);

            }

            // Save button state for next loop
            lastX = x;
        }

        public void adjustLowVelocity(boolean dpadUp, boolean dpadDown) {
            if (dpadUp && !lastDpadUp) {
                lowVelocity += 10;
            }

            if (dpadDown && !lastDpadDown) {
                lowVelocity -= 10;
            }
        }

        public double getTicksPerSec() {
            return m1.getVelocity();
        }

        public double getRPM() {
            return ((getTicksPerSec() / encoderCPM) * 60) / gearRatio;
        }

        public void telemetryOutput() {
            telemetry.addData("Target Velocity", targetRpm);
        }
    }
    private mecanumdrive drive = new mecanumdrive();

    @Override
    public void init() {
        drive.init(this);

        telemetry.addLine("Field Centric Drive Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        drive.setFieldCentricPower(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // Hold left trigger for slow mode
        drive.slowMode(gamepad1.left_trigger > 0.1);

        // Press OPTIONS to reset field heading
        if (gamepad1.options) {
            drive.resetHeading();
        }

        drive.telemetryOutput();
        telemetry.update();
    }
}

