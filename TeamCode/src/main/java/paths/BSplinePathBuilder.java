package paths;

import java.util.ArrayList;
import java.util.function.Function;
import paths.geometry.BSpline;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import util.Angle;
import util.Pose;
import util.FilletPose;
import util.Vector;

/**
 * A builder class designed to construct a {@link BSplinePath} fluently.
 * <p>
 * This class keeps track of the robot's state (its last known pose) to automatically
 * link segments together, ensuring continuous paths without needing to manually
 * pass the start point for every new curve.
 * C2 (tangent and acceleration) continuity is guaranteed in this builder
 * <p>
 * Author: DrPixelCat
 * @author Sohum Arora 22985 Paraducks
 */
public class BSplinePathBuilder {
    private final BSplinePath path;
    private Pose lastPose;
    private boolean isPointsAlreadyCreated = false;
    private static final InterpolationStyle DEFAULT_INTERPOLATION = InterpolationStyle.SMOOTH_START_TO_END;
    private InterpolationStyle currentStyle = DEFAULT_INTERPOLATION;

    public enum SegmentType {
        BSPLINE,
        TURN
    }

    /**
     * Initializes the PathBuilder with the starting location and heading of the robot.
     *
     * @param startPose The initial Pose of the robot at the beginning of the path.
     */
    public BSplinePathBuilder(Pose startPose) {
        this.path = new BSplinePath();
        this.lastPose = startPose;
    }

    /**
     * Monolithic routing method to unpack and execute multiple path steps sequentially,
     * processing embedded heading overrides completely inside the block.
     */

    /**
     * Appends a continuous Uniform Cubic B-Spline to the path.
     * The curve automatically begins at the end of the previous segment (or the start pose).
     * By default, this segment will use {@link InterpolationStyle#TANGENT_OPTIMAL} for heading.
     *
     * @param poses A variable number of waypoints to define the B-Spline curve.
     * The final pose determines the target heading for the default interpolator.
     * @return The current PathBuilder instance for method chaining.
     * @throws IllegalArgumentException If too few points are provided to construct a valid B-Spline.
     */
    public BSplinePathBuilder curveTo(Pose... poses) {
        if (poses.length < 2)
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");

        Vector[] vectors = new Vector[poses.length + 1];
        vectors[0] = lastPose.toVec();

        boolean intermediateWarningSent = false;

        for (int i = 0; i < poses.length; i++) {
            vectors[i + 1] = poses[i].toVec();

            // If an intermediate pose (not the last one) has a defined heading, throw a warning
            if (i < poses.length - 1 && !intermediateWarningSent && Double.isFinite(poses[i].getHeading())) {
                // TODO: Decide what "TBD" is
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are ignored! Only the " +
                        "final pose heading controls the end heading. (Disable this warning in TBD)");
                intermediateWarningSent = true;
            }
        }

        Pose endPose = poses[poses.length - 1];
        PathSegment curve = new PathSegment(new BSpline(vectors));

        path.addSegment(curve, buildSafeInterpolator(lastPose, endPose));

        lastPose = endPose;
        return this;
    }

    /**
     * Overrides the heading interpolation strategy for the most recently added segment.
     * This is designed to be chained immediately after adding a segment (e.g., `.lineTo(...).interpolateWith(...)`).
     *
     * @param interpolator The custom HeadingInterpolator to apply to the preceding segment.
     * @return The current PathBuilder instance for method chaining.
     */
    private BSplinePathBuilder interpolateWith(HeadingInterpolator interpolator) {
        path.overrideLastInterpolator(interpolator);
        return this;
    }

    /**
     * Easier method to call which uses interpolatePreviousSegment
     * Usage: .interpolatePreviousSegment(InterpolationStyle.TANGENT_FORWARD) instead of .interpolatePreviousSegment(new HeadingInterpolator(s -> new Angle(s * (6 * Math.PI))))
     * @param style is the style of interpolation
     * @return overrides the previous segment with selected style of interpolation
     */
    public BSplinePathBuilder interpolateWith(InterpolationStyle style) {
        return interpolateWith(new HeadingInterpolator(style));
    }

    public BSplinePathBuilder interpolateWith(Function<Double, Angle> function) {
        return interpolateWith(new HeadingInterpolator(function));
    }

    /**
     * Appends a stationary point-turn to the path.
     * The robot will stay at its current (x, y) coordinate and rotate to the target heading.
     *
     * @param targetHeading The Angle the robot should turn to face.
     * @return The current PathBuilder instance for method chaining.
     */
    public BSplinePathBuilder turnTo(Angle targetHeading) {
        path.addTurn(targetHeading);

        // Update the state tracker so the next segment knows our new heading!
        lastPose = new Pose(lastPose.getX(), lastPose.getY(), targetHeading.getRad());

        return this;
    }

    /**
     * Seamlessly holds the robot's last pose for a specific duration
     * @param durationSeconds - Duration for which pose is held (IN SECONDS)
     */
    public BSplinePathBuilder holdPose(double durationSeconds) {
        path.addHold(lastPose, durationSeconds);
        return this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) heading interpolation strategy for the whole path.
     * For fastest results, use the default for shorter segments and TANGENT_OPTIMAL for longer ones.
     *
     * @param style The style to apply to the whole path
     * @return The current PathBuilder instance for method chaining.
     */
    public BSplinePathBuilder setInterpolationStyle(InterpolationStyle style) {
        switch (style) {
            case TANGENT_OPTIMAL:
            case TANGENT_FORWARD:
            case SMOOTH_START_TO_END:
                currentStyle = style;
                break;
            default:
                throw new IllegalArgumentException(
                        "You need more parameters for: " + style.name() + "! You can use this style " +
                                "on specific segments with interpolatePreviousSegmentWith(<HeadingInterpolator>)");
        }
        return this;
    }

    /**
     * Finalizes the construction process and returns the completed path._
     *
     * @return The fully constructed {@link BSplinePath} object ready for execution._
     */
    public BSplinePath build() {
        return path;
    }

    // region Helpers

    /**
     * Safely constructs a HeadingInterpolator, automatically falling back to TANGENT_FORWARD
     * and generating a warning if a user forgot to supply valid headings in their Poses.
     */
    private HeadingInterpolator buildSafeInterpolator(Pose start, Pose end) {
        if (currentStyle == InterpolationStyle.TANGENT_FORWARD) {
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        boolean missingHeading = !Double.isFinite(start.getHeading()) || !Double.isFinite(end.getHeading());

        if (missingHeading) {
            path.addWarning("APEX WARNING: Segment missing start/end heading! Falling back to TANGENT_FORWARD. Use Pose(x, y, heading) to fix this.");
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        return new HeadingInterpolator(currentStyle, start.getHeadingComponent(), end.getHeadingComponent());
    }

    /**
     * Appends a continuous Uniform Cubic B-Spline to the path using the specified control points,
     * automatically calculating fillets for any sharp corners.
     * <p>
     * This is a fluent pre-processor and alias for {@link #curveTo(Pose...)}.
     *
     * @param poses A variable number of waypoints/control points to define the B-Spline curve.
     * The final pose determines the target heading for the default interpolator.
     * @return The current PathBuilder instance for method chaining.
     * @throws IllegalArgumentException If fewer than 2 points are provided, if endpoints are tightened,
     * or if a tightening radius geometrically exceeds adjacent segment bounds.
     */
    public BSplinePathBuilder addControlPoints(Pose... poses) {
        if (isPointsAlreadyCreated) {
            throw new IllegalArgumentException("Only one set of control points can be added to a path!");
        }
        isPointsAlreadyCreated = true;

        if (poses[0] instanceof FilletPose || poses[poses.length - 1] instanceof FilletPose) {
            throw new IllegalArgumentException("Endpoints can't be filleted any!");
        }

        // Initialize a list with enough initial capacity to hold the expanded points
        ArrayList<Pose> newPoses = new ArrayList<>(poses.length * 2);
        newPoses.add(poses[0]);

        for (int i = 1; i < poses.length - 1; i++) {
            if (poses[i] instanceof FilletPose) {
                FilletPose tightPose = (FilletPose) poses[i];
                double radius = tightPose.getRadius();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("Fillet radius must be at least 2.0 inches.");
                }

                Pose lastPose = poses[i - 1];
                Pose nextPose = poses[i + 1];

                // 1. Get raw direction vectors pointing from the corner towards the adjacent poses
                Vector vecToLast = lastPose.toVec().subtract(tightPose.toVec());
                Vector vecToNext = nextPose.toVec().subtract(tightPose.toVec());

                double distToLast = vecToLast.getMagnitude();
                double distToNext = vecToNext.getMagnitude();

                // 2. Validate that the radius does not physically exceed the adjacent segment lengths
                if (radius > distToLast) {
                    throw new IllegalArgumentException("Fillet radius (" + radius + ") exceeds the distance to the last control point.");
                } else if (radius > distToNext) {
                    throw new IllegalArgumentException("Fillet radius (" + radius + ") exceeds the distance to the next control point.");
                }

                // 3. Scale the direction vectors to be exactly 'radius' long, then add back to the corner's origin
                Vector p1Vec = tightPose.toVec().add(vecToLast.multiply(radius / distToLast));
                Vector p2Vec = tightPose.toVec().add(vecToNext.multiply(radius / distToNext));

                // 4. Inject the two newly calculated offset points instead of the original sharp corner
                newPoses.add(new Pose(p1Vec.getX(), p1Vec.getY(), tightPose.getHeading()));
                newPoses.add(new Pose(p2Vec.getX(), p2Vec.getY(), tightPose.getHeading()));

            } else {
                // Standard pose, just add it directly
                newPoses.add(poses[i]);
            }
        }

        newPoses.add(poses[poses.length - 1]);

        return this.curveTo(newPoses.toArray(new Pose[0]));
    }
}