
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

    // Heading offset used when the driver resets the field heading
    private double headingOffset = 0.0;

    // Joystick deadband
    private static final double DEADZONE = 0.05;


    /**
     * Initializes the mecanum drive.
     *
     * @param opMode The OpMode creating this drive system.
     */
    public void init(@NonNull OpMode opMode) {

        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // Get motors using the exact configured names.
        frontRightDrive = hardwareMap.get(DcMotor.class, "Front Left");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "Front Right");
        backRightDrive = hardwareMap.get(DcMotor.class, "Back Left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "Back Right");

        // Get the Control Hub IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        /*
         * Control Hub orientation.
         *
         * This assumes:
         * - The Control Hub is mounted flat.
         * - The USB ports face the front of the robot.
         * - and yes you are lying to the robot, but it works
         * If your Control Hub is mounted differently, these two directions
         * will need to be changed.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        // Motor directions.
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Use encoders for velocity feedback.
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop quickly when power is set to zero.
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start with the current robot orientation as field heading zero.
        resetHeading();
    }


    /**
     * Field-centric mecanum drive.
     *
     * @param x   Left/right movement from the left joystick.
     * @param y   Forward/backward movement from the left joystick.
     * @param rot Rotation from the right joystick.
     */
    public void setFieldCentricPower(double x, double y, double rot) {

        // Apply joystick deadzone.
        x = applyDeadzone(x);
        y = applyDeadzone(y);
        rot = applyDeadzone(rot);

        /*
         * FTC gamepad Y is negative when pushed forward,
         * so invert it here.
         */

        // y = -y;

        // Get robot heading in radians.
        double botHeading = getHeading();

        /*
         * Rotate the joystick vector by the robot's heading.
         *
         * This converts driver commands from robot-relative
         * coordinates into field-relative coordinates.
         */
        double rotX = x * Math.cos(botHeading)
                - y * Math.sin(botHeading);

        double rotY = x * Math.sin(botHeading)
                + y * Math.cos(botHeading);

        /*
         * Mecanum wheels generally need slightly more power
         * when strafing because of imperfect wheel geometry.
         */
        rotX *= 1.1;

        // Send the field-centric values to the mecanum drive.
        setPower(rotX, rotY, rot);
    }


    /**
     * Robot-centric mecanum drive.
     *
     * This can still be used if field-centric mode is not wanted.
     */
    public void setPower(double x, double y, double rot) {

        /*
         * Normalize motor powers so none exceed 1.0.
         */
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rot),
                1.0
        );

        /*
         * Motor equations for the current motor directions.
         */
        frontRightPower = (-x - y - rot) / denominator;
        frontLeftPower = (x - y + rot) / denominator;
        backRightPower = (x - y - rot) / denominator;
        backLeftPower = (-x - y + rot) / denominator;

        /*
         * Apply overall speed and slow-mode multiplier.
         */
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
     *
     * @param enable true = slow mode, false = normal speed.
     */
    public void slowMode(boolean enable) {
        slowSpeed = enable ? 0.50 : 1.0;
    }


    /**
     * Sets the maximum overall drive speed.
     *
     * @param speed Value from 0.0 to 1.0.
     */
    public void setMaxSpeed(double speed) {
        totalSpeed = Math.max(0.0, Math.min(1.0, speed));
    }


    /**
     * Returns the robot's current field-relative heading.
     *
     * @return Heading in radians.
     */
    public double getHeading() {

        double rawHeading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        return rawHeading - headingOffset;
    }


    /**
     * Resets the current robot orientation to field heading zero.
     *
     * This does NOT reset the physical IMU. It simply establishes
     * the robot's current heading as the new field-forward direction.
     */
    public void resetHeading() {

        headingOffset = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }


    /**
     * Resets the actual IMU yaw.
     */
    public void resetIMU() {
        imu.resetYaw();
        headingOffset = 0.0;
    }


    /**
     * Switches the wheels to run using encoders.
     */
    public void runUsingEncoder() {

        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Switches the wheels to run without encoders.
     */
    public void runWithoutEncoder() {

        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Switches the wheels to RUN_TO_POSITION.
     */
    public void runToPosition() {

        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**
     * Applies a joystick deadzone.
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
}

