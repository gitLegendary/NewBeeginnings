package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class shooter {

    Telemetry telemetry;
    DcMotorEx flywheelMotor;
    Gamepad gamepad2;

    private double flywheelRpm = 3000;

    // Encoder ticks per revolution for velocity control (adjust based on your flywheel motor specification)
    private static final double TICKS_PER_REV = 28.0;

    // Edge detection variables to detect "was pressed" single button presses
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    /**
     * Initializes the shooter system and automatically binds Gamepad 2.
     *
     * @param opMode The OpMode creating this shooter system.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad2 = opMode.gamepad2; // Automatically assigns Gamepad 2 from the OpMode

        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "Flywheel");
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) {
                telemetry.addData("Shooter Warning", "Motor 'Flywheel' not found in Hardware Map");
            }
        }
    }

    /**
     * Updates shooter speed automatically using Gamepad 2.
     */
    public void update() {
        if (gamepad2 != null) {
            update(gamepad2);
        }
    }

    /**
     * Updates shooter speed based on D-pad inputs and spins flywheel when Right Trigger is held.
     * - D-pad Up: Increases target RPM by 25
     * - D-pad Down: Decreases target RPM by 25
     * - Right Trigger: When held (> 0.1 threshold), spins the flywheel motor at target velocity
     *
     * @param gamepad Gamepad instance to read inputs from
     */
    public void update(@NonNull Gamepad gamepad) {
        // Detect rising edge for D-pad Up press
        if (gamepad.dpad_up && !lastDpadUp) {
            flywheelRpm += 25;
        }

        // Detect rising edge for D-pad Down press
        if (gamepad.dpad_down && !lastDpadDown) {
            flywheelRpm -= 25;
        }

        // Save D-pad states for next iteration
        lastDpadUp = gamepad.dpad_up;
        lastDpadDown = gamepad.dpad_down;

        // Spin flywheel motor when right trigger is held
        if (flywheelMotor != null) {
            if (gamepad.right_trigger > 0.1) {
                // Convert target RPM to ticks per second for closed-loop encoder control
                double ticksPerSecond = (flywheelRpm * TICKS_PER_REV) / 60.0;
                flywheelMotor.setVelocity(ticksPerSecond);
            } else {
                // Stop the flywheel when right trigger is released
                flywheelMotor.setPower(0);
            }
        }
    }

    public double getFlywheelRpm() {
        return flywheelRpm;
    }

    public void setFlywheelRpm(double flywheelRpm) {
        this.flywheelRpm = flywheelRpm;
    }

    /**
     * Displays flywheel telemetry information.
     */
    public void telemetryOutput() {
        if (telemetry != null) {
            telemetry.addData("Flywheel Target RPM", flywheelRpm);
            if (flywheelMotor != null) {
                telemetry.addData("Flywheel Active", flywheelMotor.getPower() != 0 || flywheelMotor.getVelocity() != 0);
            }
        }
    }
}
