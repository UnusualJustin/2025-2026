package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.config.IntakeConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public final class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    private final MotorEx intakeMotor = new MotorEx("intake");

    public void on() {
        intakeMotor.setPower(IntakeConfig.onPower);
    }

    public void off() {
        intakeMotor.setPower(IntakeConfig.offPower);
    }

    @Override
    public void periodic() {
        // no periodic logic needed
    }
}

