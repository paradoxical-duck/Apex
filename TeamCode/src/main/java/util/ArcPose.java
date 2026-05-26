package util;

/**
 * Represents a specialized waypoint in a path that should be smoothed into an arc.
 * <p>
 * When passed into a path builder, an {@code ArcPose} acts as a marker indicating that
 * the sharp corner at this location should be dynamically split and rounded.
 * The {@code radius} defines how far from the corner the smoothing begins and ends.
 * @author DrPixelCat
 */
public class ArcPose extends Pose {

    private final double radius;

    /**
     * Constructs an {@code ArcPose} by inheriting the coordinates, heading, and units
     * of a base pose, while attaching a specified corner radius.
     * * @param basePose The original waypoint representing the sharp corner.
     * @param radius   The radius of the arc used to smooth the corner (in the same distance units as the base pose).
     */
    public ArcPose(Pose basePose, double radius) {
        super(basePose.getX(), basePose.getY(), basePose.getHeading(),
                basePose.getDistanceUnit(), basePose.getAngleUnit(), false);

        Distance.from(basePose.getDistanceUnit(), radius);

        this.radius = radius;
    }

    /**
     * Gets the geometric radius of the arc used to smooth this waypoint's corner.
     * * @return The smoothing radius.
     */
    public double getRadius() {
        return radius;
    }
}