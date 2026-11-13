
package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

public class mecanumdrive {

    final DecimalFormat df = new DecimalFormat("0.00");

    // Drive motors
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    // IMU
    IMU imu;

    // Telemetry
    Telemetry telemetry;

    // Motor powers
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    private double backLeftPower;

    // Speed settings
    private double totalSpeed = 1.0;
    private double slowSpeed = 1.0;

    // Heading offset
    private double headingOffset = 0.0;

    // Joystick deadband
    private static final double DEADZONE = 0.05;


    /**
     * Initializes the mecanum drive.
     */
    public void init(@NonNull OpMode opMode) {

        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // Get motors using their configured names.
        frontRightDrive = hardwareMap.get(DcMotor.class, "Front Right");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "Front Left");
        backRightDrive = hardwareMap.get(DcMotor.class, "Back Right");
        backLeftDrive = hardwareMap.get(DcMotor.class, "Back Left");

        // Get the Control Hub IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        /*
         * Control Hub orientation:
         * Logo faces UP.
         * USB ports face FORWARD.
         */
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        // Motor directions.
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Use encoders.
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when stopped.
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Establish an initial heading reference.
        resetHeading();
    }


    /**
     * Field-centric mecanum drive.
     */
    public void setFieldCentricPower(double x, double y, double rot) {

        // Apply joystick deadzone.
        x = applyDeadzone(x);
        y = applyDeadzone(y);
        rot = applyDeadzone(rot);

        // FTC gamepad Y is negative when pushed forward.
        y = -y;

        // Get heading relative to the reset reference.
        double botHeading = getHeading();

        // Rotate joystick input by robot heading.
        double rotX = x * Math.cos(-botHeading)
                - y * Math.sin(-botHeading);

        double rotY = x * Math.sin(-botHeading)
                + y * Math.cos(-botHeading);

        // Strafing correction.
        rotX *= 1.1;

        // Apply motor powers.
        setPower(rotX, rotY, rot);
    }


    /**
     * Robot-centric mecanum drive.
     */
    public void setPower(double x, double y, double rot) {

        // Normalize motor powers.
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rot),
                1.0
        );

        // Motor equations.
        frontRightPower = (-x - y - rot) / denominator;
        frontLeftPower = (x - y + rot) / denominator;
        backRightPower = (x - y - rot) / denominator;
        backLeftPower = (-x - y + rot) / denominator;

        // Apply speed settings.
        frontRightDrive.setPower(
                frontRightPower * totalSpeed * slowSpeed
        );

        frontLeftDrive.setPower(
                frontLeftPower * totalSpeed * slowSpeed
        );

        backRightDrive.setPower(
                backRightPower * totalSpeed * slowSpeed
        );

        backLeftDrive.setPower(
                backLeftPower * totalSpeed * slowSpeed
        );
    }


    /**
     * Enables or disables slow mode.
     */
    public void slowMode(boolean enable) {
        slowSpeed = enable ? 0.50 : 1.0;
    }


    /**
     * Sets maximum drive speed.
     */
    public void setMaxSpeed(double speed) {
        totalSpeed = Math.max(0.0, Math.min(1.0, speed));
    }


    /**
     * Returns heading relative to the reset reference.
     */
    public double getHeading() {

        double rawHeading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        double heading = rawHeading - headingOffset;

        // Keep heading between -PI and PI.
        while (heading > Math.PI) {
            heading -= 2.0 * Math.PI;
        }

        while (heading < -Math.PI) {
            heading += 2.0 * Math.PI;
        }

        return heading;
    }


    /**
     * Sets the current robot orientation to heading zero.
     */
    public void resetHeading() {

        headingOffset = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }


    /**
     * Resets the physical IMU yaw.
     */
    public void resetIMU() {

        imu.resetYaw();
        headingOffset = 0.0;
    }


    /**
     * Switches wheels to RUN_USING_ENCODER.
     */
    public void runUsingEncoder() {

        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Switches wheels to RUN_WITHOUT_ENCODER.
     */
    public void runWithoutEncoder() {

        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Switches wheels to RUN_TO_POSITION.
     */
    public void runToPosition() {

        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**
     * Applies joystick deadzone.
     */
    private double applyDeadzone(double value) {

        if (Math.abs(value) < DEADZONE) {
            return 0.0;
        }

        return value;
    }


    /**
     * Sends drive information to telemetry.
     */
    public void telemetryOutput() {

        telemetry.addData(
                "Heading",
                "%.1f°",
                Math.toDegrees(getHeading())
        );

        telemetry.addData(
                "Front Right Power",
                df.format(frontRightPower)
        );

        telemetry.addData(
                "Front Left Power",
                df.format(frontLeftPower)
        );

        telemetry.addData(
                "Back Right Power",
                df.format(backRightPower)
        );

        telemetry.addData(
                "Back Left Power",
                df.format(backLeftPower)
        );

        telemetry.addData(
                "Front Left Ticks",
                frontLeftDrive.getCurrentPosition()
        );

        telemetry.addData(
                "Front Right Ticks",
                frontRightDrive.getCurrentPosition()
        );

        telemetry.addData(
                "Back Left Ticks",
                backLeftDrive.getCurrentPosition()
        );

        telemetry.addData(
                "Back Right Ticks",
                backRightDrive.getCurrentPosition()
        );
    }


    public void stop() {
        setPower(0, 0, 0);
    }
}
