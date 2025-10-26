package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Launcher {
    public DcMotorEx motor1, motor2;

    private Telemetry telemetry;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void updateInputs(boolean a, boolean b, double rpm) {
        double ticksPerRev = 383.6;//motor1.getMotorType().getTicksPerRev();
        double ticksPerSecond = (rpm * ticksPerRev) / 60;

        if (a) {
            motor1.setVelocity(ticksPerSecond);
            motor2.setVelocity(ticksPerSecond);
        } else if (b) {
            motor1.setVelocity(0);
            motor2.setVelocity(0);
        }

        if (motor1.getPower() > 0) {
            motor1.setVelocity(ticksPerSecond);
            motor2.setVelocity(ticksPerSecond);
        }

        telemetry.addData("Motor 1 Power:", motor1.getPower());

        telemetry.addData("Motor 2 Power:", motor2.getPower());
    }
}
