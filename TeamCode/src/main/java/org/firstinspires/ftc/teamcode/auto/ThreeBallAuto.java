package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GoalConfig;
import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.PosePublisher;
import org.firstinspires.ftc.teamcode.subsystems.commands.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.targeting.DistanceProvider;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 3 balls", group = "auto")
public class ThreeBallAuto extends NextFTCOpMode {
    public final static class AutoPaths {

        private final double shootingAngle = 165; //deg
        private final Pose blueStartingPose = new Pose(25, 127, Math.toRadians(145));
        private final Pose shortShootingPose = new Pose(55, 127, Math.toRadians(shootingAngle));

        public final AutoPathSpec shootPreloadPath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(90), Math.toRadians(shootingAngle));

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private Flywheel flywheel;
    private Paddle paddle;
    private Intake intake;
    private Follower follower;

    @Override
    public void onInit() {
        super.onInit();

        paddle = new Paddle();
        intake = new Intake();
        follower = AutoConstants.createFollower(hardwareMap);
        DistanceProvider distanceProvider = new DistanceProvider(follower);
        flywheel = new Flywheel(distanceProvider);

        addComponents(
                new SubsystemComponent(
                        flywheel,
                        paddle,
                        intake,
                        new PosePublisher(follower)
                ),
                new PedroComponent((hardwareDevices -> follower))
        );

        paddle.lower.run();
    }

    private Command autoRoutine() {
        AutoPaths paths = new AutoPaths();

        follower.setStartingPose(paths.getStartingPose());

        return new SequentialGroup(
                paddle.lower,

                // Enable distance-based flywheel RPM
                new InstantCommand(flywheel::enableAutoFromDistance),

                // Move forward to shoot preloaded balls
                new FollowPath(paths.shootPreloadPath.build(follower, GoalConfig.goal)),

                // Shoot preloaded balls
                shootCommand(),
                new InstantCommand(intake::on),
                new Delay(0.25),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                // Stop flywheel after shooting
                new InstantCommand(flywheel::stop),
                new InstantCommand(intake::off)
        );
    }

    private SequentialGroup shootCommand() {
        return new SequentialGroup(
                new WaitUntilCommand(flywheel::isAtSpeed),
                paddle.feedOnce());
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke();
    }
}
