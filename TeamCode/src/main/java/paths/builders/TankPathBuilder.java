package paths.builders;

import java.util.ArrayList;
import java.util.List;

import core.FollowerConstants;
import feedforward.FeedforwardLut;
import feedforward.tank.TankProfileGenerator;
import geometry.Angle;
import geometry.ArcPose;
import geometry.BSpline;
import geometry.Dist;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.callbacks.Callback;
import paths.constraint.PathConstraint;
import paths.heading.TankInterpolationStyle;
import paths.heading.TankInterpolator;
import paths.movements.Path;

/**
 * A builder class designed to construct a {@link Path} fluently strictly for Tank drivetrains.
 * Because tank drives cannot strafe, heading interpolation is automatically locked to the path
 * tangent.
 */
public class TankPathBuilder {
    public Path path;
    private final Pose[] rawPoses;
    private TankInterpolationStyle style = TankInterpolationStyle.TANGENT_FORWARD;
    private final List<Runnable> buildTasks = new ArrayList<>();

    /**
     * Creates a new TankPathBuilder using the provided poses.
     *
     * @param poses A sequence of Pose objects defining the path. Must contain at least two poses
     *              . Endpoints cannot be ArcPoses.
     */
    public TankPathBuilder(Pose... poses) {
        this.path = new Path(Path.PathType.TANK);
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
        this.rawPoses = poses;
    }

    /**
     * Overrides the default interpolation style for the tank drive.
     *
     * @param style The TankInterpolationStyle to apply.
     * @return The current TankPathBuilder instance for method chaining.
     */
    public TankPathBuilder interpolateWith(TankInterpolationStyle style) {
        this.style = style;
        return this;
    }

    /**
     * Adds a kinematic constraint to the path at a specific distance percentage.
     *
     * @param constraint The {@link PathConstraint} to be added to the path
     * @return The current TankPathBuilder instance for method chaining.
     */
    public TankPathBuilder addConstraint(PathConstraint constraint) {
        if (constraint.getS() >= 1.0 || constraint.getS() < 0.0) {
            constraint.setS(Math.min(Math.max(constraint.getS(), 0.0), 0.9));
            path.addWarning("s must be within [0, 1) bounds! Normalized to " + constraint.getS() +
                    " for safety.");
        }
        path.addConstraint(constraint);
        return this;
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s      The physical distance percentage [0.0, 1.0].
     * @param action The code to execute.
     * @return The current TankPathBuilder instance for method chaining.
     */
    public TankPathBuilder addDistanceCallback(double s, Runnable action) {
        buildTasks.add(() -> path.addCallback(new Callback(s, action)));
        return this;
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     *
     * @param angle  The Angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current TankPathBuilder instance for method chaining.
     */
    public TankPathBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> path.addCallback(new Callback(angle, action)));
        return this;
    }

    /**
     * Internal method to compile the B-Spline geometry, process arc poses, and initialize the
     * tank interpolator.
     */
    private void compileGeometry() {
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 " +
                            "inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.getVec().minus(arcPose.getVec());
                Vector vecToNext = nextPose.getVec().minus(arcPose.getVec());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast || radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius exceeds distance to " +
                            "adjacent control points.");
                }

                Vector p1Vec = arcPose.getVec().plus(vecToLast.times(radius / distToLast));
                Vector p2Vec = arcPose.getVec().plus(vecToNext.times(radius / distToNext));

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
            vectors[i] = processedPoses.get(i).getVec();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));
        path.setParametricPath(curve);

        TankInterpolationStyle resolvedStyle = style;
        Vector startTangent = curve.getFirstDerivative(0.0);

        if (resolvedStyle == TankInterpolationStyle.TANGENT_OPTIMAL) {
            Angle startHeading = rawPoses[0].getHeading();
            double fwdError =
                    Math.abs(startHeading.getShortestAngleTo(startTangent.getTheta()).getRad());
            double bwdError =
                    Math.abs(startHeading.getShortestAngleTo(startTangent.getTheta().plus(Angle.fromRad(Math.PI))).getRad());
            resolvedStyle = (bwdError < fwdError) ? TankInterpolationStyle.TANGENT_BACKWARD :
                    TankInterpolationStyle.TANGENT_FORWARD;
        }

        TankInterpolator interpolator = new TankInterpolator(resolvedStyle);
        path.setInterpolator(interpolator);

        Vector finalVec = vectors[vectors.length - 1];
        Vector finalTangent = curve.getFirstDerivative(1.0);
        Angle finalHeading = finalTangent.getTheta();
        if (resolvedStyle == TankInterpolationStyle.TANGENT_BACKWARD) {
            finalHeading = finalHeading.plus(Angle.fromRad(Math.PI));
        }

        path.setEndPose(new Pose(finalVec, finalHeading));

        for (Runnable task : buildTasks) {
            task.run();
        }
    }

    /**
     * Builds the path geometry and generates a naive trapezoidal profile.
     * Used to quickly satisfy the mathematical requirements of a Ramsete controller without
     * heavy computation.
     *
     * @return The constructed Path with a basic FeedforwardLut attached.
     */
    public Path quickBuild() {
        compileGeometry();
        FollowerConstants config = new FollowerConstants();
        TankProfileGenerator generator = new TankProfileGenerator(config, path);

        if (path.getConstraints().length == 0) {
            path.addWarning("APEX WARNING: quickBuild() called on Tank drive with no constraints!" +
                    " The naive profile will attempt maximum speed through all curves.");
        }

        path.setFeedforwardLut(generator.generateQuick(config));
        return path;
    }

    /**
     * Builds the path geometry and solves a complete kinematically constrained feedforward
     * motion profile.
     *
     * @return The constructed Path with a fully optimized FeedforwardLut attached.
     */
    public Path profiledBuild() {
        compileGeometry();
        FollowerConstants config = new FollowerConstants();
        TankProfileGenerator generator = new TankProfileGenerator(config, path);

        path.setFeedforwardLut(new FeedforwardLut(generator.generate()));
        return path;
    }
}