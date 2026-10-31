package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class flywheelpid {

    private DcMotorEx m1;

    // Encoder and gearing
    private double encoderCPM = 28;
    private double gearRatio = 3.0 / 2.0;

    // Flywheel control coefficients
    private double kV = 0.000171;
    private double kS = 0.09;
    private double kP = 0.0009;

    // Normal flywheel target RPM
    public double lowVelocity = 2750;

    // Reverse flywheel target RPM
    private double reverseRPM = 50;

    private double targetRpm = 0;

    private Telemetry telemetry;

    private boolean lastX = false;
    private boolean flywheelToggle = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    /**
     * Initialize the flywheel motor.
     */
    public void init(@NonNull OpMode opMode) {

        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        m1 = hardwareMap.get(
                DcMotorEx.class,
                "Turret Launcher"
        );

        // Use encoder-based motor control.
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start with the motor stopped.
        m1.setPower(0);
    }

    /**
     * Set motor power directly.
     */
    public void setMotorPower(double power) {

        power = Range.clip(power, -1.0, 1.0);

        m1.setPower(power);
    }

    /**
     * Run the flywheel using the normal control system.
     *
     * leftTrigger:
     *     Runs at lowVelocity when pressed.
     *
     * rightTrigger:
     *     Retained for the original control system.
     *
     * x:
     *     Toggles distance-based velocity mode.
     *
     * distance:
     *     Distance used to calculate the target RPM.
     */
    public void setMotorRPM(
            float leftTrigger,
            float rightTrigger,
            boolean x,
            double distance
    ) {

        // Toggle automatic velocity mode when X is newly pressed.
        if (x && !lastX) {
            flywheelToggle = !flywheelToggle;
        }

        if (leftTrigger > 0.5 && rightTrigger < 0.5) {

            // Normal low-velocity mode.
            targetRpm = lowVelocity;

            double error = targetRpm - getRPM();

            double ff = (kV * targetRpm) + kS;
            double fb = error * kP;

            double power = ff + fb;

            setMotorPower(power);

        } else if (rightTrigger > 0.5 && leftTrigger < 0.5) {

            // Original right-trigger behavior.
            // The TeleOp reverse control bypasses this branch.
            double error = targetRpm - getRPM();

            double ff = (kV * targetRpm) + kS;
            double fb = error * kP;

            double power = ff + fb;

            setMotorPower(power);

        } else if (flywheelToggle) {

            // Calculate target RPM from distance.
            targetRpm = Range.clip(
                    -0.0366411 * Math.pow(distance, 2)
                            + 21.04711 * distance
                            + 707.85114,
                    0,
                    4000
            );

            double error = targetRpm - getRPM();

            double ff = (kV * targetRpm) + kS;
            double fb = error * kP;

            double power = ff + fb;

            setMotorPower(power);

        } else {

            // Stop when no flywheel mode is active.
            targetRpm = 0;

            setMotorPower(0);
        }

        // Save button state for the next loop.
        lastX = x;
    }

    /**
     * Run the flywheel in reverse at the requested RPM.
     *
     * A negative velocity tells the motor to rotate
     * in the opposite direction.
     */
    public void runReverse(double targetReverseRPM) {

        // Convert the requested RPM to encoder ticks per second.
        double targetTicksPerSecond =
                -targetReverseRPM * encoderCPM * gearRatio / 60.0;

        targetRpm = -targetReverseRPM;

        // Command a negative velocity using the motor encoder.
        m1.setVelocity(targetTicksPerSecond);
    }

    /**
     * Adjust the normal flywheel target RPM.
     *
     * Each D-pad press changes the target by 10 RPM.
     */
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

    /**
     * Return encoder velocity in ticks per second.
     */
    public double getTicksPerSec() {
        return m1.getVelocity();
    }

    /**
     * Convert encoder velocity into RPM.
     */
    public double getRPM() {

        return (
                (getTicksPerSec() / encoderCPM) * 60.0
        ) / gearRatio;
    }

    /**
     * Display flywheel telemetry.
     */
    public void telemetryOutput() {

        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Current RPM", getRPM());
        telemetry.addData("Low Velocity Setting", lowVelocity);
    }

    /**
     * Stop the flywheel.
     */
    public void stop() {

        targetRpm = 0;

        if (m1 != null) {
            m1.setPower(0);
        }
    }
}