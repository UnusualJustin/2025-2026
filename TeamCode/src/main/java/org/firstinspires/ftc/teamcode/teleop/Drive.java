package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class Drive extends OpMode {
    private DriveController driveController;
    private Launcher launcher;
    private final Gamepad previousGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    public static double RPM = 225;

    public static double paddlePos = .56;

    private int loopCount = 0;
    private boolean raised = false;
    Servo paddleServo;

    public static int loopThreshold = 25;

    @Override
    public void init() {
        driveController = new DriveController(hardwareMap, telemetry);
        paddleServo = hardwareMap.get(Servo.class, "paddleServo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        launcher = new Launcher(hardwareMap, telemetry);
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
        //driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);
        driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x* -1);


        launcher.updateInputs(gamepad1.a, gamepad1.b, RPM);
        previousGamepad2.copy(gamepad2);
        previousGamepad1.copy(gamepad1);

        if (previousGamepad1.yWasPressed()) {
            paddleServo.setPosition(paddlePos);
            loopCount = 0;
            raised = true;
        }

        if (raised) {
            loopCount ++;
            if (loopCount > loopThreshold ) {
                raised =false;
                paddleServo.setPosition(0.5);
            }
        }
        telemetry.addData("loopcount" , loopCount);
        telemetry.update();
    }
}