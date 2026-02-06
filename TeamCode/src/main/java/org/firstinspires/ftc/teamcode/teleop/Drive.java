package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.config.FlywheelConfig;
import org.firstinspires.ftc.teamcode.targeting.AimingCalculator;
import org.firstinspires.ftc.teamcode.targeting.DistanceProvider;
import org.firstinspires.ftc.teamcode.targeting.TeamConfig;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Drive", group = "teleop")
public class Drive extends NextFTCOpMode {

    public static Pose startingPose = new Pose(72, 72, Math.toRadians(90));
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.2;

    private static final double STICK_DEAD_ZONE = 0.07;

    // ------------------- Hold / Input state -------------------
    private Pose aimPose = new Pose();
    private boolean aimRequested = false;

    // Snapshot of where we expected to be when we started aiming
    private Pose aimAnchorPose = new Pose();

    private boolean wasDriverInput = true;

    private boolean idleHoldActive = false;
    private boolean aimHoldActive = false;

    private boolean pendingIdleHold = false;
    private final ElapsedTime idleSettleTimer = new ElapsedTime();

    // Tuning knobs
    private static final double IDLE_SETTLE_SEC = 0.15;          // delay before capturing idle pose
    private static final double TURN_CANCEL_THRESHOLD = 0.02;    // "really zero" turn after deadzone

    private static final double AIM_HEADING_TOL_RAD = Math.toRadians(2.0); // 1–3 deg typical
    private static final double AIM_ANCHOR_TOL_IN = 2.0;                   // inches drift allowed

    // ------------------- Endgame timers -------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean didRumble145 = false;
    private boolean didShutdown155 = false;

    private static final double RUMBLE_TIME_SEC = 105.0;   // 1:45
    private static final double SHUTDOWN_TIME_SEC = 115.0; // 1:55

    public Drive() {
        addComponents(
                // NextFTC runtime plumbing
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,

                // Subsystems
                new SubsystemComponent(Flywheel.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Kickstand.INSTANCE),
                new SubsystemComponent(Paddle.INSTANCE),

                // Pedro integration: creates + updates follower automatically
                new PedroComponent(Constants::createFollower));
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(startingPose == null ? new Pose() : startingPose);

        telemetryM.update(telemetry);
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();

        if (gamepad1.backWasPressed()) {
            TeamConfig.goal = TeamConfig.goal == AimingCalculator.Goal.BLUE_GOAL ?
                    AimingCalculator.Goal.RED_GOAL : AimingCalculator.Goal.BLUE_GOAL;
        }

        telemetryM.debug("goal", TeamConfig.goal);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().startTeleopDrive();

        // Start as "input present" so we don't immediately auto-hold on start
        wasDriverInput = true;

        idleHoldActive = false;
        aimHoldActive = false;
        pendingIdleHold = false;

        Flywheel.INSTANCE.enableAutoFromDistance();

        matchTimer.reset();
        didRumble145 = false;
        didShutdown155 = false;
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);

        // Read sticks with deadzone first
        double driveY = applyDeadZone(-gamepad1.left_stick_y);
        double driveX = applyDeadZone(-gamepad1.left_stick_x);
        double turn = applyDeadZone(-gamepad1.right_stick_x);

        boolean driverInputDetected = (Math.abs(driveY) > 0.0) || (Math.abs(driveX) > 0.0) || (Math.abs(turn) > 0.0);

        // ------------------- Cancel holds on driver input -------------------
        if (driverInputDetected) {
            // Any stick motion cancels pending idle hold immediately
            pendingIdleHold = false;

            // It also cancels any pending shot
            aimRequested = false;

            // If we were previously "no input", then this is the transition back to manual
            if (!wasDriverInput) {
                idleHoldActive = false;
                aimHoldActive = false;

                // Cancel hold by returning to teleop drive mode
                PedroComponent.follower().startTeleopDrive();
            }
        }

        // ------------------- Begin idle settle when sticks go neutral --------
        if (!driverInputDetected && wasDriverInput) {
            pendingIdleHold = true;
            idleSettleTimer.reset();

            // ensure we stop commanding motion during settle
            PedroComponent.follower().setTeleOpDrive(0, 0, 0, true);
        }

        // ------------------- Commit idle hold after settle delay -------------
        if (!driverInputDetected && pendingIdleHold) {
            boolean turnIsReallyZero = Math.abs(turn) <= TURN_CANCEL_THRESHOLD;

            if (turnIsReallyZero && idleSettleTimer.seconds() >= IDLE_SETTLE_SEC) {
                Pose idleHoldPose = PedroComponent.follower().getPose(); // capture AFTER settle
                idleHoldActive = true;
                aimHoldActive = false;
                pendingIdleHold = false;

                PedroComponent.follower().holdPoint(idleHoldPose);
            }
        }

        wasDriverInput = driverInputDetected;

        // ------------------- TeleOp drive when driver is commanding ----------
        if (driverInputDetected) {
            if (slowMode) {
                driveY *= slowModeMultiplier;
                driveX *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }

            PedroComponent.follower().setTeleOpDrive(driveY, driveX, turn, true);
        }

        // ------------------- Aim hold (right bumper) -------------------------
        if (gamepad1.rightBumperWasPressed()) {
            aimPose = AimingCalculator.computeAimPose(PedroComponent.follower().getPose(), AimingCalculator.Goal.BLUE_GOAL);

            aimRequested = true;

            aimHoldActive = true;
            idleHoldActive = false;
            pendingIdleHold = false;

            // Anchor where we were when we requested the shot
            aimAnchorPose = PedroComponent.follower().getPose();

            PedroComponent.follower().holdPoint(aimPose);
        }

        // ------------------- Fire only when aimed + flywheel is ready --------
        if (aimRequested) {
            Pose currentPose = PedroComponent.follower().getPose();

            boolean headingGood = angleAbsDiffRad(currentPose.getHeading(), aimPose.getHeading()) <= AIM_HEADING_TOL_RAD;

            // "Still valid" = haven't been shoved (position drift), NOT heading drift
            boolean anchorStillValid = distanceInches(currentPose, aimAnchorPose) <= AIM_ANCHOR_TOL_IN;

            if (headingGood && anchorStillValid && Flywheel.INSTANCE.isAtSpeed()) {
                CommandManager.INSTANCE.scheduleCommand(Paddle.INSTANCE.feedOnce());
                aimRequested = false;
            }
        }

        // ------------------- Slow Mode toggle --------------------------------
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // ------------------- Intake ------------------------------------------
        if (gamepad1.aWasPressed()) {
            Intake.INSTANCE.on();
        } else if (gamepad1.bWasPressed()) {
            Intake.INSTANCE.off();
        }

        // ------------------- Flywheel target velocity ------------------------
        if (gamepad1.dpadUpWasPressed()) {
            Flywheel.INSTANCE.setTargetRpm(FlywheelConfig.targetRpm + 50);
        } else if (gamepad1.dpadDownWasPressed()) {
            Flywheel.INSTANCE.setTargetRpm((Math.max(FlywheelConfig.targetRpm - 50, 0)));
        }

        // ------------------- Auto adjust speed for flywheel ------------------
        if (gamepad1.dpadLeftWasPressed()) {
            Flywheel.INSTANCE.disableAutoFromDistance();
        } else if (gamepad1.dpadRightWasPressed()) {
            Flywheel.INSTANCE.enableAutoFromDistance();
        }

        // ------------------- Kickstand ---------------------------------------
        if (gamepad1.xWasPressed()) {
            cancelHolds();
            Kickstand.INSTANCE.deploy();
        } else if (gamepad1.yWasPressed()) {
            Kickstand.INSTANCE.retract();
        }

        double elapsedSec = matchTimer.seconds();

        // Vibrate at 1:45 (once)
        if (!didRumble145 && elapsedSec >= RUMBLE_TIME_SEC) {
            gamepad1.rumbleBlips(5);
            didRumble145 = true;
        }

        // Shutdown at 1:55 (once)
        if (!didShutdown155 && elapsedSec >= SHUTDOWN_TIME_SEC) {
            Flywheel.INSTANCE.stop();
            Intake.INSTANCE.off();
            didShutdown155 = true;
        }

        telemetryM.debug("driverInputDetected", driverInputDetected);
        telemetryM.debug("pendingIdleHold", pendingIdleHold);
        telemetryM.debug("idleHoldActive", idleHoldActive);
        telemetryM.debug("aimHoldActive", aimHoldActive);
        telemetryM.debug("target rpm", Flywheel.INSTANCE.getTargetRpm());
        telemetryM.debug("actual rpm", Flywheel.INSTANCE.getCurrentRpm());
        telemetryM.debug("distance from target", DistanceProvider.INSTANCE.getDistance());
        telemetryM.debug("kickstand position", hardwareMap.get(DcMotorEx.class, "kickstand"));
    }

    @Override
    public void onStop() {
        super.onStop();
        Kickstand.INSTANCE.retract();
    }

    private double applyDeadZone(double value) {
        if (Math.abs(value) < STICK_DEAD_ZONE) {
            return 0.0;
        }
        return value;
    }

    private static double angleAbsDiffRad(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2.0 * Math.PI;
        while (diff < -Math.PI) diff += 2.0 * Math.PI;
        return Math.abs(diff);
    }

    private static double distanceInches(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    private void cancelHolds() {
        pendingIdleHold = false;
        idleHoldActive = false;
        aimHoldActive = false;

        aimRequested = false;

        // Return follower to normal teleop control (cancels holdPoint)
        PedroComponent.follower().startTeleopDrive();
    }

}
