package org.firstinspires.ftc.teamcode.targeting;

import com.acmerobotics.roadrunner.Pose2d;

public final class AimingTargetCalculator {

    // ------------------------------------------------------------
    // GOAL ENUM
    // ------------------------------------------------------------

    public enum Goal {
        BLUE_GOAL,   // (0, 144)
        RED_GOAL     // (144, 144)
    }

    // ------------------------------------------------------------
    // BASE GOAL COORDINATES
    // ------------------------------------------------------------

    private static final double BLUE_GOAL_X = 0.0;
    private static final double RED_GOAL_X  = 144.0;

    // Both goals share the same Y coordinate at the top of the field
    private static final double GOAL_Y = 144.0;

    // ------------------------------------------------------------
    // OFFSET CONFIGURATION
    // ------------------------------------------------------------

    // Field midpoint (assuming 144" × 144" and (72,72) is center)
    private static final double FIELD_CENTER = 72.0;

    // How far inward we are willing to shift the shot target (inches)
    private static final double OFFSET_MAX = 12.0;

    private AimingTargetCalculator() {
        // Prevent instantiation — static utility
    }

    // ------------------------------------------------------------
    // PUBLIC API
    // ------------------------------------------------------------

    /**
     * Computes the field-centric desired heading in degrees [0, 360)
     * based on robot position and selected goal.
     *
     * @param currentPose The current robot position on the field.
     * @param goal which goal to aim at (BLUE_GOAL or RED_GOAL)
     */
    public static Pose2d computeAimPose(Pose2d currentPose, Goal goal) {
        double baseGoalX = (goal == Goal.BLUE_GOAL) ? BLUE_GOAL_X : RED_GOAL_X;

        double targetX = computeDynamicGoalX(currentPose.getX(), baseGoalX);
        double targetY = computeDynamicGoalY(currentPose.getY());

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        double headingRad = Math.atan2(deltaY, deltaX);
        double headingDeg = Math.toDegrees(headingRad);

        if (headingDeg < 0) {
            headingDeg += 360.0;
        }
        return headingDeg;
    }

    // ------------------------------------------------------------
    // INTERNAL COMPUTATION
    // ------------------------------------------------------------

    /**
     * Computes a dynamic X target:
     * - For BLUE_GOAL (0,144) we pull inward toward +X (0 → 12).
     * - For RED_GOAL  (144,144) we pull inward toward -X (144 → 132).
     *
     * The inward offset is scaled based on how far the robot is from the goal
     * along the X-axis, with more offset when further from field center.
     */
    private static double computeDynamicGoalX(double robotX, double baseGoalX) {
        double distanceFromGoalX = Math.abs(baseGoalX - robotX);

        // Factor in [0, 1]: 0 = no offset, 1 = max inward offset
        double offsetFactor = (FIELD_CENTER - distanceFromGoalX);
        offsetFactor = clamp(offsetFactor, 0.0, FIELD_CENTER) / FIELD_CENTER;

        double inwardOffset = offsetFactor * X_OFFSET_MAX;

        // Apply inward toward the field center
        if (baseGoalX == BLUE_GOAL_X) {
            // Blue goal: move rightward (toward +X)
            return baseGoalX + inwardOffset;
        } else {
            // Red goal: move leftward (toward –X)
            return baseGoalX - inwardOffset;
        }
    }

    /**
     * Computes dynamic Y target (same for both goals).
     * If the robot is above field center (robotY > FIELD_CENTER),
     * we gradually pull the target Y downward from Y_TARGET_BASE
     * to Y_TARGET_MIN.
     */
    private static double computeDynamicGoalY(double robotY) {
        double offsetFactor = (robotY - FIELD_CENTER);
        offsetFactor = clamp(offsetFactor, 0.0, FIELD_CENTER) / FIELD_CENTER;

        return interpolateLinear(GOAL_Y, GOAL_Y - OFFSET_MAX, offsetFactor);
    }

    // ------------------------------------------------------------
    // UTILITY FUNCTIONS
    // ------------------------------------------------------------

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    /**
     * Simple linear interpolation between start and end.
     * interpolationFactor is assumed to be in [0, 1].
     */
    private static double interpolateLinear(
            double start,
            double end,
            double interpolationFactor
    ) {
        return start + (end - start) * interpolationFactor;
    }
}
