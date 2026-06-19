package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import paths.movements.Path;
import paths.callbacks.Callback;
import geometry.BSpline;
import geometry.PathSegment;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import geometry.Angle;
import geometry.Vector;
import geometry.ArcPose;
import geometry.Pose;

/**
 * A builder class designed to construct a {@link Path} fluently.
 * This class captures path configurations (waypoints, interpolators, callbacks)
 * in any order and defers geometric compilation until {@link #build()} is called.
 * C2 (tangent and acceleration) continuity is guaranteed in this builder.
 * @author Sohum Arora 22985 Paraducks
 * @author DrPixelCat
 */
public class PathBuilder {
    public Path path;

    private Pose expectedEndPose;
    private Pose[] rawPoses = null;

    private InterpolationStyle currentStyle = InterpolationStyle.SMOOTH_START_TO_END;
    private Angle customOffset = null;
    private Function<Double, Angle> customFunction = null;

    private final List<Runnable> buildTasks = new ArrayList<>();

    protected PathBuilder(Pose... poses) {
        this.path = new Path();
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
        this.rawPoses = poses;
        this.startPose = poses[0];
        this.expectedEndPose = poses[poses.length - 1];
    }

    private final Pose startPose;

    /**
     * Overrides the default (SMOOTH_START_TO_END) interpolation with a different {@link InterpolationStyle}
     *
     * @param style The interpolation style to apply.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(InterpolationStyle style) {
        this.currentStyle = style;
        return this;
    }

    /**
     * Overrides the interpolation style, providing a custom angular offset.
     * Used primarily for {@link InterpolationStyle#TANGENT_CUSTOM}.
     *
     * @param style The interpolation style to apply.
     * @param angleOffset The fixed angle to offset the calculation by.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(InterpolationStyle style, Angle angleOffset) {
        this.currentStyle = style;
        this.customOffset = angleOffset;
        return this;
    }

    /**
     * Overrides the default interpolation with a custom function of distance percentage (s).
     *
     * @param function A lambda mapping distance percentage [0.0, 1.0] to a target Angle.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(Function<Double, Angle> function) {
        this.currentStyle = InterpolationStyle.CUSTOM_DIST_FUNCTION;
        this.customFunction = function;
        return this;
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s The physical distance percentage [0.0, 1.0].
     * @param action The code to execute.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder addDistanceCallback(double s, Runnable action) {
        buildTasks.add(() -> {
            path.addCallback(new Callback(s, action));
        });
        return this;
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     *
     * @param angle The Angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            if (currentStyle == InterpolationStyle.SMOOTH_START_TO_END) {
                Angle startRad = rawPoses[0].getHeading();
                Angle endRad = expectedEndPose.getHeading();

                if (Double.isFinite(startRad.getRad()) && Double.isFinite(endRad.getRad())) {
                    double totalDiff = startRad.getShortestAngleTo(endRad).getRad();
                    double targetDiff = startRad.getShortestAngleTo(angle).getRad();

                    if (Math.abs(totalDiff) < 1e-6) {
                        if (Math.abs(targetDiff) > 1e-6) {
                            throw new IllegalArgumentException("Angular callback out of bounds: The path's target heading is constant.");
                        }
                    } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                        throw new IllegalArgumentException("Angular callback is outside the sweep range of the start and end headings.");
                    }
                }
            }
            path.addCallback(new Callback(angle, action));
        });
        return this;
    }

    /**
     * Compiles all configuration data, calculates new ctrl points from {@link ArcPose}, generates the curve,
     * verifies callback safety, and returns the completed executable Path.
     *
     * @return The fully constructed {@link Path} object ready for execution.
     */
    public Path build() {
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading().getRad())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are currently ignored! Only the " +
                        "final pose heading controls the end heading.");
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.getPos().minus(arcPose.getPos());
                Vector vecToNext = nextPose.getPos().minus(arcPose.getPos());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the last control point.");
                } else if (radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the next control point.");
                }

                Vector p1Vec = arcPose.getPos().plus(vecToLast.times(radius / distToLast));
                Vector p2Vec = arcPose.getPos().plus(vecToNext.times(radius / distToNext));

                processedPoses.add(new Pose(p1Vec, arcPose.getHeading()));
                processedPoses.add(currentPose);
                processedPoses.add(new Pose(p2Vec, arcPose.getHeading()));

            } else {
                processedPoses.add(currentPose);
            }
        }

        processedPoses.add(rawPoses[rawPoses.length - 1]);

        Vector[] vectors = new Vector[processedPoses.size()];
        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i] = processedPoses.get(i).getPos();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));
        path.setParametricPath(curve);

        Angle startH = startPose.getHeading();
        Angle endH = expectedEndPose.getHeading();

        boolean missingParams =
                (currentStyle == InterpolationStyle.CONSTANT_START_HEADING && !Double.isFinite(startH.getRad())) ||
                        (currentStyle == InterpolationStyle.CONSTANT_END_HEADING && !Double.isFinite(endH.getRad())) ||
                        (currentStyle == InterpolationStyle.TANGENT_CUSTOM && (customOffset == null || !Double.isFinite(customOffset.getRad()))) ||
                        (currentStyle == InterpolationStyle.SMOOTH_START_TO_END && (!Double.isFinite(startH.getRad()) || !Double.isFinite(endH.getRad()))) ||
                        (currentStyle == InterpolationStyle.CUSTOM_DIST_FUNCTION && customFunction == null);

        if (missingParams) {
            path.addWarning("APEX WARNING: " + currentStyle.name() + " is missing required parameters! Falling back to TANGENT_FORWARD.");
            currentStyle = InterpolationStyle.TANGENT_FORWARD;
        }

        path.setInterpolator(new HeadingInterpolator(currentStyle, startH, endH, customOffset));
        path.setEndPose(expectedEndPose);

        for (Runnable task : buildTasks) {
            task.run();
        }

        return path;
    }

}