package paths.heading;

/**
 * Defines the strategy used to calculate the robot's target heading (orientation)
 * at any given distance percentage along a path segment.
 * Author: DrPixelCat
 */
public enum TankInterpolationStyle {

    /**
     * The robot aligns its heading with the tangent of the path, but chooses the
     * direction (forward, backward) that requires the least amount of rotation.
     */
    TANGENT_OPTIMAL,

    /**
     * The robot strictly faces the forward direction of travel along the path
     * at all times
     */
    TANGENT_FORWARD,

    /**
     * The robot strictly faces the reverse direction of travel along the path
     * at all times
     */
    TANGENT_BACKWARD
}