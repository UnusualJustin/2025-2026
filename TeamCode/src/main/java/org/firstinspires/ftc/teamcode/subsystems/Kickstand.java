package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.config.KickstandConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public final class Kickstand implements Subsystem {

    public static final Kickstand INSTANCE = new Kickstand();

    private Kickstand() {}

    private final MotorEx kickstandMotor = new MotorEx("kickstand");

    public void deploy() {
        moveTo(KickstandConfig.deployTicks);
    }

    public void retract() {
        moveTo(KickstandConfig.retractTicks);
    }

    public boolean isBusy() {
        return kickstandMotor.getMotor().isBusy();
    }

    private void moveTo(int ticks) {
        DcMotorEx motor = kickstandMotor.getMotor();

        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(KickstandConfig.movePower);
    }

    @Override
    public void initialize() {
        Subsystem.super.initialize();

        DcMotorEx motor = kickstandMotor.getMotor();

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
    }
}
