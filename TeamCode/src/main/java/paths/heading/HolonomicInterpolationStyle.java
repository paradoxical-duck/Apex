package paths.heading;

/**
 * Defines the strategy used to calculate the robot's target heading (orientation)
 * at any given distance percentage along a path segment.
 * Author: DrPixelCat
 */
public enum HolonomicInterpolationStyle {

    /**
     * The robot locks its heading to whatever the heading was at the start of the segment.
     * Useful for pure strafing moves where the robot should not rotate.
     */
    CONSTANT_START_HEADING,

    /**
     * The robot immediately targets and locks to the heading defined at the end of the segment.
     */
    CONSTANT_END_HEADING,

    /**
     * The robot aligns its heading with the tangent of the path, but chooses the
     * direction (forward, backward) that requires the least amount of rotation.
     */
    TANGENT_OPTIMAL,

    /**
     * The robot strictly faces the forward direction of travel along the path
     * at all times, driving like a standard car.
     */
    TANGENT_FORWARD,

    /**
     * The robot follows the tangent of the path but with a fixed angular offset
     * (e.g., pointing 90 degrees away from the path).
     */
    TANGENT_CUSTOM,

    /**
     * The robot smoothly interpolates its heading from the starting angle to the
     * ending angle as it travels the segment (usually a linear interpolation based on 't').
     */
    SMOOTH_START_TO_END,

    /**
     * The heading is controlled by custom user-defined nodes along the curve,
     * allowing for complex heading profiles completely independent of the path's shape.
     */
    NODE_BASED,

    /**
     * The robot faces a fixed field point for the full path, with an optional angular offset
     * for aiming a side or mechanism toward that point.
     */
    FACING_POINT
}
