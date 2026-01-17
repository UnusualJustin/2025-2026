package org.firstinspires.ftc.teamcode.pidTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TeleOp for live-tuning the launcher flywheel velocity controller.
 *
 * Tuning inputs come from LauncherFlywheelTuning (Panels).
 * - targetVelocity (ticks/sec)
 * - pid (PIDCoefficients)
 * - ffV/ffA/ffS
 * - toleranceVelocity
 */
@TeleOp(name = "TUNE: Launcher Flywheel Velocity", group = "Tuning")
@Configurable
public class TuneLauncher extends LinearOpMode {

    private TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.addData("Target", 0);
        panelsTelemetry.addData("Error", 0);
        panelsTelemetry.addData("Measured", 0);

        panelsTelemetry.update();

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");

        // We are doing our own velocity control -> avoid built-in velocity mode.
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LauncherFlywheelController controller = new LauncherFlywheelController(flywheelMotor);

        waitForStart();

        while (opModeIsActive()) {
            // Optional: quick driver controls while tuning
            // - A: spin at current targetVelocity (Panels)
            // - B: stop
            if (gamepad1.b) {
                controller.stop();
            }

            // Run tuning controller continuously
            controller.update();

            double measuredVel = flywheelMotor.getVelocity();
            double targetVel = LauncherFlywheelTuning.targetVelocity;
            double error = targetVel - measuredVel;

            panelsTelemetry.addData("Target", targetVel);
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("Measured", measuredVel);

            panelsTelemetry.update();
        }

        controller.stop();
    }
}

