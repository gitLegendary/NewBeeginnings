package org.firstinspires.ftc.teamcode.init;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.action.mecanumdrive;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {

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

