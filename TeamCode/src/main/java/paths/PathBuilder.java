package paths;

import java.util.ArrayList;
import java.util.function.Function;
import paths.geometry.BSpline;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import util.Angle;
import util.Pose;
import util.ArcPose;
import util.Vector;

/**
 * A builder class designed to construct a {@link Path} fluently.
 * <p>
 * This class keeps track of the robot's state (its last known pose) to automatically
 * link segments together, ensuring continuous paths without needing to manually
 * pass the start point for every new curve.
 * C2 (tangent and acceleration) continuity is guaranteed in this builder
 * <p>
 * @author DrPixelCat
 * @author Sohum Arora 22985 Paraducks
 */
public class PathBuilder {
    public Path path;
    private Pose lastPose;
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
    public PathBuilder(Pose startPose) {
        this.path = new Path();
        this.lastPose = startPose;
    }

    /**
     * Appends a continuous Uniform Cubic B-Spline to the path using the specified control points.
     * The curve automatically begins at the end of the previous segment (or the start pose).
     * <p>
     * Any {@link ArcPose} provided in the sequence is dynamically split into two adjacent
     * control points based on its specified radius, seamlessly smoothing out sharp corners.
     *
     * @param poses A variable number of waypoints/control points to define the B-Spline curve.
     * The final pose determines the target heading for the default interpolator.
     * @return The current PathBuilder instance for method chaining.
     * @throws IllegalArgumentException If fewer than 2 points are provided, if endpoints are arc poses,
     * or if an arc pose radius geometrically exceeds adjacent segment bounds.
     */
    public PathBuilder addControlPoints(Pose... poses) {
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }

        // 1. Pre-process the points (Expand ArcPoses and check for invalid headings)
        ArrayList<Pose> processedPoses = new ArrayList<>(poses.length * 2);
        processedPoses.add(poses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < poses.length - 1; i++) {
            Pose currentPose = poses[i];

            // Warning for intermediate headings
            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are ignored! Only the " +
                        "final pose heading controls the end heading.");
                intermediateWarningSent = true;
            }

            // Expand ArcPoses into two separate points
            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 inches.");
                }

                Pose prevPose = poses[i - 1];
                Pose nextPose = poses[i + 1];

                Vector vecToLast = prevPose.toVec().subtract(arcPose.toVec());
                Vector vecToNext = nextPose.toVec().subtract(arcPose.toVec());

                double distToLast = vecToLast.getMagnitude();
                double distToNext = vecToNext.getMagnitude();

                if (radius > distToLast) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the last control point.");
                } else if (radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the next control point.");
                }

                Vector p1Vec = arcPose.toVec().add(vecToLast.multiply(radius / distToLast));
                Vector p2Vec = arcPose.toVec().add(vecToNext.multiply(radius / distToNext));

                processedPoses.add(new Pose(p1Vec.getX(), p1Vec.getY(), arcPose.getHeading()));
                processedPoses.add(new Pose(p2Vec.getX(), p2Vec.getY(), arcPose.getHeading()));

            } else {
                processedPoses.add(currentPose);
            }
        }

        processedPoses.add(poses[poses.length - 1]);

        // 2. Build the curve using the fully processed points
        Vector[] vectors = new Vector[processedPoses.size() + 1];
        vectors[0] = lastPose.toVec(); // Inherit end of previous segment

        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i + 1] = processedPoses.get(i).toVec();
        }

        Pose endPose = processedPoses.get(processedPoses.size() - 1);
        PathSegment curve = new PathSegment(new BSpline(vectors));

        // 3. Inject segment and update state
        path.addSegment(curve, buildSafeInterpolator(lastPose, endPose));
        lastPose = endPose;

        return this;
    }

    /**
     * Overrides the heading interpolation strategy for the most recently added segment.
     * This is designed to be chained immediately after adding a segment.
     *
     * @param interpolator The custom HeadingInterpolator to apply to the preceding segment.
     * @return The current PathBuilder instance for method chaining.
     */
    private PathBuilder interpolateWith(HeadingInterpolator interpolator) {
        path.overrideLastInterpolator(interpolator);
        return this;
    }

    /**
     * Easier method to call which uses interpolatePreviousSegment
     * Usage: .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
     * @param style is the style of interpolation
     * @return overrides the previous segment with selected style of interpolation
     */
    public PathBuilder interpolateWith(InterpolationStyle style) {
        return interpolateWith(new HeadingInterpolator(style));
    }

    public PathBuilder interpolateWith(Function<Double, Angle> function) {
        return interpolateWith(new HeadingInterpolator(function));
    }

    /**
     * Appends a stationary point-turn to the path.
     * The robot will stay at its current (x, y) coordinate and rotate to the target heading.
     *
     * @param targetHeading The Angle the robot should turn to face.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder turnTo(Angle targetHeading) {
        path.addTurn(targetHeading);

        // Update the state tracker so the next segment knows our new heading!
        lastPose = new Pose(lastPose.getX(), lastPose.getY(), targetHeading.getRad());

        return this;
    }

    /**
     * Seamlessly holds the robot's last pose for a specific duration
     * @param durationSeconds - Duration for which pose is held (IN SECONDS)
     */
    public PathBuilder holdPose(double durationSeconds) {
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
    public PathBuilder setInterpolationStyle(InterpolationStyle style) {
        switch (style) {
            case TANGENT_OPTIMAL:
            case TANGENT_FORWARD:
            case SMOOTH_START_TO_END:
                currentStyle = style;
                break;
            default:
                throw new IllegalArgumentException(
                        "You need more parameters for: " + style.name() + "! You can use this style " +
                                "on specific segments with interpolateWith(<HeadingInterpolator>)");
        }
        return this;
    }

    /**
     * Finalizes the construction process and returns the completed path.
     *
     * @return The fully constructed {@link Path} object ready for execution.
     */
    public Path build() {
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
     * Attaches an executable callback to the most recently added segment.
     * @param s The physical distance percentage [0.0, 1.0] along the segment at which to trigger the callback.
     * @param callback The code to execute (e.g., moving an arm, opening a claw).
     * @return The current PathBuilder instance for method chaining.
     *
     * TODO: Xenon plz review this I just made something rq
     */
    public PathBuilder addCallback(double s, Runnable callback) {
        double clampedS = Math.max(0.0, Math.min(1.0, s));
        path.withCallback(clampedS, callback);
        return this;
    }
}