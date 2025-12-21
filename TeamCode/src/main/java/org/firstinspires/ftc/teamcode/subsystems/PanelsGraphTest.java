package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test Graph")
public class PanelsGraphTest extends OpMode {
    private TelemetryManager PanelsTelemetry = new TelemetryManager(); // Ensure getter matches your library
    private final ElapsedTime timer = new ElapsedTime();

    private double sinVariable = 0.0;
    private double cosVariable = 0.0;
    private double constVariable = 0.0;
    private double dampedSine = 0.0;
    private double lissajous = 0.0;
    private double ramp = 0.0;
    private double squareWave = 0.0;

    @Override
    public void init() {
        timer.reset();
        updateSignals();
    }

    @Override
    public void loop() {
        updateSignals();
    }

    private void updateSignals() {
        double t = timer.seconds();

        sinVariable = Math.sin(t);
        cosVariable = Math.cos(t);
        constVariable = 1.0;

        dampedSine = Math.exp(-0.2 * t) * Math.sin(2 * t);

        lissajous = Math.sin(3 * t + Math.PI / 2) * Math.cos(2 * t);

        ramp = (t % 5.0) / 5.0;

        squareWave = (Math.sin(2 * Math.PI * 0.5 * t) > 0) ? 1.0 : -1.0;

        PanelsTelemetry.addData("sin", sinVariable);
        PanelsTelemetry.addData("cos", cosVariable);
        PanelsTelemetry.addData("dampedSine", dampedSine);
        PanelsTelemetry.addData("ramp", ramp);
        PanelsTelemetry.addData("lissajous", lissajous);
        PanelsTelemetry.addData("square", squareWave);
        PanelsTelemetry.addData("const", constVariable);

        PanelsTelemetry.addLine("extra1:" + t + " extra2:" + (t * t) + " extra3:" + Math.sqrt(t));

        PanelsTelemetry.update();
    }
}
