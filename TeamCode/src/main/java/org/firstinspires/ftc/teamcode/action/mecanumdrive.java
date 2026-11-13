
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

    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    IMU imu;
    Telemetry telemetry;

    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    private double backLeftPower;

    private double totalSpeed = 1.0;
    private double slowSpeed = 1.0;

    private double headingOffset = 0.0;

    private static final double DEADZONE = 0.05;

    public void init(@NonNull OpMode opMode) {

        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        frontRightDrive = hardwareMap.get(DcMotor.class, "Front Right");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "Front Left");
        backRightDrive = hardwareMap.get(DcMotor.class, "Back Right");
        backLeftDrive = hardwareMap.get(DcMotor.class, "Back Left");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetHeading();
    }

    public void setFieldCentricPower(double x, double y, double rot) {

        x = applyDeadzone(x);
        y = applyDeadzone(y);
        rot = applyDeadzone(rot);

        double botHeading = getHeading();

        // Rotate joystick input using the robot heading
        double rotX = x * Math.cos(botHeading)
                - y * Math.sin(botHeading);

        double rotY = x * Math.sin(botHeading)
                + y * Math.cos(botHeading);

        setPower(rotX, rotY, rot);
    }

    public void setPower(double x, double y, double rot) {

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rot),
                1.0
        );

        frontRightPower = (-x - y - rot) / denominator;
        frontLeftPower = (x - y + rot) / denominator;
        backRightPower = (x - y - rot) / denominator;
        backLeftPower = (-x - y + rot) / denominator;

        frontRightDrive.setPower(frontRightPower * totalSpeed * slowSpeed);
        frontLeftDrive.setPower(frontLeftPower * totalSpeed * slowSpeed);
        backRightDrive.setPower(backRightPower * totalSpeed * slowSpeed);
        backLeftDrive.setPower(backLeftPower * totalSpeed * slowSpeed);
    }

    public void slowMode(boolean enable) {
        slowSpeed = enable ? 0.5 : 1.0;
    }

    public void setMaxSpeed(double speed) {
        totalSpeed = Math.max(0.0, Math.min(1.0, speed));
    }

    public double getHeading() {

        double rawHeading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        return rawHeading - headingOffset;
    }

    public void resetHeading() {

        headingOffset = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }

    public void resetIMU() {
        imu.resetYaw();
        headingOffset = 0.0;
    }

    public void runUsingEncoder() {
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition() {
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE) {
            return 0.0;
        }
        return value;
    }

    public void telemetryOutput() {

        telemetry.addData("Heading", "%.1f°",
                Math.toDegrees(getHeading()));

        telemetry.addData("Front Right Power", df.format(frontRightPower));
        telemetry.addData("Front Left Power", df.format(frontLeftPower));
        telemetry.addData("Back Right Power", df.format(backRightPower));
        telemetry.addData("Back Left Power", df.format(backLeftPower));

        telemetry.addData("Front Left Ticks", frontLeftDrive.getCurrentPosition());
        telemetry.addData("Front Right Ticks", frontRightDrive.getCurrentPosition());
        telemetry.addData("Back Left Ticks", backLeftDrive.getCurrentPosition());
        telemetry.addData("Back Right Ticks", backRightDrive.getCurrentPosition());
    }

    public void stop() {
    }
}
