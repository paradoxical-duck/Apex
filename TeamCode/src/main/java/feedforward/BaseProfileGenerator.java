package feedforward;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import core.FollowerConstants;
import geometry.PathPoint;
import geometry.Vector;
import paths.constraint.PathConstraint;
import paths.movements.FollowerMovement;
import paths.movements.Path;

public abstract class BaseProfileGenerator {

    protected FollowerMovement path;
    private ElapsedTime timer = new ElapsedTime();
    private DebugReport lastReport;

    public DebugReport getLastDebugReport() {
        return lastReport;
    }

    // region Abstract Methods
    protected abstract double calculateMaxTangentialVelocity(PathPoint point, PathPoint lastPoint, Path path, double maxAngVel, double maxAngAccel);

    protected abstract void evaluatePoint(
            Path path, PathPoint prev, PathPoint current,
            double v_prev, double v, double a_t,
            EvaluationResult outResult
    );

    protected abstract double getMaxTangentialAccel(double currentVel, PathPoint point, Path path, double maxAngAccel);

    protected abstract double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path, double maxAngAccel);

    // region Master Loop

    public MotionParameters[] generate() {
        if (!(path instanceof Path)) {
            throw new IllegalArgumentException("BaseFeedforwardGen only handles Path movements.");
        }

        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();
        if (points == null || points.length == 0) {
            throw new IllegalStateException("Points must be set before generating.");
        }

        lastReport = new DebugReport();

        MotionParameters[] outputParams = generateBasePass(points, path);
        runBackwardPass(outputParams, points, path);
        runForwardPass(outputParams, points, path);

        int generationTime_ms = 750;
        int iter = 0;
        timer.reset();
        EvaluationResult currentEval = new EvaluationResult();
        EvaluationResult worstEvalState = new EvaluationResult();

        while (timer.time(TimeUnit.MILLISECONDS) < generationTime_ms) {
            lastReport.iterationsRun = iter + 1;

            double[] utilizations = new double[points.length];
            double globalWorstUtil = 0.0;
            int globalWorstIndex = -1;

            for (int i = 1; i < points.length; i++) {
                double ds = Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
                if (ds < 1e-6) {
                    utilizations[i] = 0.0;
                    continue;
                }

                double v = outputParams[i].getTangentialVel();
                double v_prev = outputParams[i - 1].getTangentialVel();
                double a_t = ((v * v) - (v_prev * v_prev)) / (2.0 * ds);

                evaluatePoint(path, points[i - 1], points[i], v_prev, v, a_t, currentEval);
                utilizations[i] = currentEval.maxUtilization;

                if (currentEval.maxUtilization > globalWorstUtil) {
                    globalWorstUtil = currentEval.maxUtilization;
                    globalWorstIndex = i;
                    worstEvalState.copyFrom(currentEval);
                }
            }

            lastReport.finalMaxUtilization = globalWorstUtil;

            if (globalWorstUtil <= 1.0) {
                lastReport.converged = true;
                break;
            }

            boolean pinnedAny = false;
            for (int i = 1; i < points.length; i++) {
                if (utilizations[i] > 1.0) {
                    boolean isLocalMax = true;
                    if (i > 1 && utilizations[i] < utilizations[i - 1]) isLocalMax = false;
                    if (i < points.length - 1 && utilizations[i] <= utilizations[i + 1]) isLocalMax = false;

                    if (isLocalMax) {
                        outputParams[i].setTangentialVel(outputParams[i].getTangentialVel() * 0.90);
                        pinnedAny = true;
                    }
                }
            }

            if (!pinnedAny && globalWorstIndex != -1) {
                outputParams[globalWorstIndex].setTangentialVel(outputParams[globalWorstIndex].getTangentialVel() * 0.90);
            }

            IterationLog log = new IterationLog();
            log.iteration = iter;
            log.pinnedIndex = globalWorstIndex;
            log.maxUtilization = worstEvalState.maxUtilization;
            log.totalPower = worstEvalState.totalPower;
            log.pForward = worstEvalState.pForward;
            log.pLateral = worstEvalState.pLateral;
            log.pHeading = worstEvalState.pHeading;
            lastReport.logs.add(log);

            runBackwardPass(outputParams, points, path);
            runForwardPass(outputParams, points, path);

            iter++;
        }

        return outputParams;
    }

    public FeedforwardLut generateQuick(FollowerConstants config) {
        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();
        MotionParameters[] lut = new MotionParameters[points.length];

        // Base pass: Max velocity everywhere
        for (int i = 0; i < points.length; i++) {
            lut[i] = new MotionParameters();
            lut[i].setTangentialVel(config.forwardVelocityLimit.getIn());
        }

        // Backward pass: Naive deceleration
        // If boosted, relax the boundary condition so the path ends at cruising speed
        if (!path.isAccelBoosted()) {
            lut[points.length - 1].setTangentialVel(0.0);
        }

        for (int i = points.length - 2; i >= 0; i--) {
            double ds = Math.abs(points[i + 1].getDistanceToEnd_in() - points[i].getDistanceToEnd_in());
            double nextVel = lut[i + 1].getTangentialVel();
            double maxReachableVel = Math.sqrt((nextVel * nextVel) + (2.0 * config.forwardAccelerationLimit.getIn() * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }

        // Forward pass: Naive acceleration and populate angular targets
        // If boosted, relax the boundary condition so the path begins at cruising speed
        if (!path.isAccelBoosted()) {
            lut[0].setTangentialVel(0.0);
        }

        for (int i = 1; i < points.length; i++) {
            double ds = Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
            double prevVel = lut[i - 1].getTangentialVel();
            double maxReachableVel = Math.sqrt((prevVel * prevVel) + (2.0 * config.forwardAccelerationLimit.getIn() * ds));

            double v = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            lut[i].setTangentialVel(v);

            double a_t = ((v * v) - (prevVel * prevVel)) / (2.0 * ds);
            lut[i].setTangentialAccel(a_t);

            double s = points[i].getDistanceToEnd_in();
            double kappa = points[i].getSignedCurvature();
            double dKappa = points[i].getCurvatureDerivative();
            Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

            lut[i].setAngularVel(fPrime * v);
            lut[i].setAngularAccel((fDoublePrime * (v * v)) + (fPrime * a_t));
        }

        return new FeedforwardLut(lut);
    }

    // region Fwd/Bkwd Passes

    private MotionParameters[] generateBasePass(PathPoint[] points, Path path) {
        MotionParameters[] lut = new MotionParameters[points.length];
        double pathLength_in = path.getParametricPath().getLengthIn();
        PathConstraint[] constraints = path.getConstraints();

        for (int i = 0; i < points.length; i++) {
            double pctCompleted = 1.0 - (points[i].getDistanceToEnd_in() / pathLength_in);
            double currentMaxVel = Double.MAX_VALUE;
            double currentMaxAngVel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.s <= pctCompleted) {
                    if (constraint.constraintType == PathConstraint.ConstraintType.VELOCITY) {
                        currentMaxVel = constraint.value_in;
                    } else if (constraint.constraintType == PathConstraint.ConstraintType.ANGULAR_VELOCITY) {
                        currentMaxAngVel = constraint.value_in;
                    } else if (constraint.constraintType == PathConstraint.ConstraintType.ANGULAR_ACCELERATION) {
                        currentMaxAngAccel = constraint.value_in;
                    }
                }
            }

            double maxVel = calculateMaxTangentialVelocity(points[i], i == 0 ? points[0] : points[i - 1], path, currentMaxAngVel, currentMaxAngAccel);
            if (currentMaxVel != Double.MAX_VALUE && currentMaxVel > 0.0) {
                maxVel = Math.min(maxVel, currentMaxVel);
            }

            lut[i] = new MotionParameters();
            lut[i].setTangentialVel(maxVel);
        }
        return lut;
    }

    private void runBackwardPass(MotionParameters[] lut, PathPoint[] points, Path path) {
        // If boosted, relax the boundary condition so the path ends at cruising speed
        if (!path.isAccelBoosted()) {
            lut[points.length - 1].setTangentialVel(0.0);
        }

        double pathLength_in = path.getParametricPath().getLengthIn();
        PathConstraint[] constraints = path.getConstraints();

        for (int i = points.length - 2; i >= 0; i--) {
            double ds = Math.abs(points[i + 1].getDistanceToEnd_in() - points[i].getDistanceToEnd_in());
            double nextVel = lut[i + 1].getTangentialVel();

            double pctCompleted = 1.0 - (points[i].getDistanceToEnd_in() / pathLength_in);
            double currentMaxAccel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.s <= pctCompleted) {
                    if (constraint.constraintType == PathConstraint.ConstraintType.ACCELERATION) {
                        currentMaxAccel = constraint.value_in;
                    } else if (constraint.constraintType == PathConstraint.ConstraintType.ANGULAR_ACCELERATION) {
                        currentMaxAngAccel = constraint.value_in;
                    }
                }
            }

            double maxAccel = getMaxTangentialAccel(nextVel, points[i], path, currentMaxAngAccel);
            if (currentMaxAccel != Double.MAX_VALUE && currentMaxAccel > 0.0) {
                maxAccel = Math.min(maxAccel, currentMaxAccel);
            }

            double maxReachableVel = Math.sqrt((nextVel * nextVel) + (2.0 * maxAccel * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }
    }

    private void runForwardPass(MotionParameters[] lut, PathPoint[] points, Path path) {
        /* If boosted, relax the boundary condition so the path begins at cruising speed so the
        feedback controller can provide maximum +/- power */
        if (!path.isAccelBoosted()) {
            lut[0].setTangentialVel(0.0);
        }

        double pathLength_in = path.getParametricPath().getLengthIn();
        PathConstraint[] constraints = path.getConstraints();

        for (int i = 1; i < points.length; i++) {
            double ds = Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
            double prevVel = lut[i - 1].getTangentialVel();

            double pctCompleted = 1.0 - (points[i].getDistanceToEnd_in() / pathLength_in);
            double currentMaxAccel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.s <= pctCompleted) {
                    if (constraint.constraintType == PathConstraint.ConstraintType.ACCELERATION) {
                        currentMaxAccel = constraint.value_in;
                    } else if (constraint.constraintType == PathConstraint.ConstraintType.ANGULAR_ACCELERATION) {
                        currentMaxAngAccel = constraint.value_in;
                    }
                }
            }

            double dynamicAccel = calculateDynamicMaxAccel(prevVel, points[i], path, currentMaxAngAccel);
            if (currentMaxAccel != Double.MAX_VALUE && currentMaxAccel > 0.0) {
                dynamicAccel = Math.min(dynamicAccel, currentMaxAccel);
            }

            double maxReachableVel = Math.sqrt((prevVel * prevVel) + (2.0 * dynamicAccel * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }
    }

    // region Logging and CSV Export

    public static class EvaluationResult {
        public double totalPower = 0.0;
        public double maxUtilization = 0.0;
        public double pForward = 0.0;
        public double pLateral = 0.0;
        public double pHeading = 0.0;

        public void copyFrom(EvaluationResult other) {
            this.totalPower = other.totalPower;
            this.maxUtilization = other.maxUtilization;
            this.pForward = other.pForward;
            this.pLateral = other.pLateral;
            this.pHeading = other.pHeading;
        }
    }

    public static class DebugReport {
        public int iterationsRun = 0;
        public boolean converged = false;
        public double finalMaxUtilization = 0.0;
        public List<IterationLog> logs = new ArrayList<>();

        public String getSummary() {
            return String.format("Converged: %b | Iterations: %d | Final Max Util: %.3f",
                    converged, iterationsRun, finalMaxUtilization);
        }
    }

    public static class IterationLog {
        public int iteration;
        public int pinnedIndex;
        public double previousVelocity, newVelocity;
        public double maxUtilization, totalPower;
        public double pForward, pLateral, pHeading;
    }
}