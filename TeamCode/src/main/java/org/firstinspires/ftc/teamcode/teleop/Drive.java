package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class Drive extends OpMode {
    private DriveController driveController;
    private final Gamepad previousGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        driveController = new DriveController(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void stop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        // multiply left stick y by -1 to invert direction (forward on the joystick gives negative values)
        driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);

        previousGamepad2.copy(gamepad2);
        previousGamepad1.copy(gamepad1);

        telemetry.update();
    }
}