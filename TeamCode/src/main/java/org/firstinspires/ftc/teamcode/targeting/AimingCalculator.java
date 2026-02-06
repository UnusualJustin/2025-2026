package org.firstinspires.ftc.teamcode.targeting;

import com.pedropathing.geometry.Pose;

public final class AimingCalculator {

    public enum Goal {
        BLUE_GOAL,   // (0, 144)
        RED_GOAL     // (144, 144)
    }

    private static final double BLUE_GOAL_X = 0.0;
    private static final double RED_GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;

    private static final double FIELD_CENTER = 72.0;
    private static final double OFFSET_MAX = 12.0;

    private AimingCalculator() {
        // static utility
    }

    /**
     * Returns the dynamically-shifted goal point (x,y) the robot should aim toward.
     * Heading is unused and set to 0.
     */
    public static Pose computeDynamicGoalPose(Pose currentPose, Goal goal) {
        double baseGoalX = (goal == Goal.BLUE_GOAL) ? BLUE_GOAL_X : RED_GOAL_X;

        double targetX = computeDynamicGoalX(currentPose.getX(), baseGoalX);
        double targetY = computeDynamicGoalY(currentPose.getY());

        return new Pose(targetX, targetY, 0.0);
    }

    /**
     * Returns a new Pose with the same x/y as the robot,
     * but a heading that aims at the dynamically-shifted goal point.
     */
    public static Pose computeAimPose(Pose currentPose, Goal goal) {
        Pose target = computeDynamicGoalPose(currentPose, goal);

        double deltaX = target.getX() - currentPose.getX();
        double deltaY = target.getY() - currentPose.getY();

        double headingRad = Math.atan2(deltaY, deltaX);

        return new Pose(
                currentPose.getX(),
                currentPose.getY(),
                headingRad
        );
    }

    public static double computeAimHeadingRad(Pose currentPose, Goal goal) {
        return computeAimPose(currentPose, goal).getHeading();
    }

    // ----------------------------
    // INTERNAL COMPUTATION
    // ----------------------------

    private static double computeDynamicGoalX(double robotX, double baseGoalX) {
        double distanceFromGoalX = Math.abs(baseGoalX - robotX);

        double offsetFactor =
                clamp(FIELD_CENTER - distanceFromGoalX, 0.0, FIELD_CENTER)
                        / FIELD_CENTER;

        double inwardOffset = offsetFactor * OFFSET_MAX;

        return (baseGoalX == BLUE_GOAL_X)
                ? baseGoalX + inwardOffset
                : baseGoalX - inwardOffset;
    }

    private static double computeDynamicGoalY(double robotY) {
        double offsetFactor =
                clamp(robotY - FIELD_CENTER, 0.0, FIELD_CENTER)
                        / FIELD_CENTER;

        return lerp(GOAL_Y, GOAL_Y - OFFSET_MAX, offsetFactor);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}
