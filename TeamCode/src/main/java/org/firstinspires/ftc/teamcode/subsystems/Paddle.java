package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.config.PaddleConfig;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public final class Paddle implements Subsystem {

    public static final Paddle INSTANCE = new Paddle();

    private Paddle() {
    }

    private final ServoEx paddleServo = new ServoEx("paddle");


    // Simple commands you can reuse elsewhere
    public final Command lower = new SetPosition(paddleServo, PaddleConfig.downPosition).requires(this);

    public final Command raise = new SetPosition(paddleServo, PaddleConfig.upPosition).requires(this);

    /**
     * One feed cycle: raise -> wait -> lower.
     */
    public Command feedOnce() {
        return new SequentialGroup(raise, new Delay(PaddleConfig.feedTimeSeconds),
                lower).requires(this);
    }

    @Override
    public void periodic() {
        // no periodic updates for servos.
    }
}

