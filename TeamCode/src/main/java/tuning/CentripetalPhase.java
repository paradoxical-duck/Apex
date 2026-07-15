package tuning;

import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

public class CentripetalPhase extends TuningPhase {
    private static final double ERROR_TARGET = 0.15;

    private final TuningValues values;
    private BinarySearch search;
    private Path[] arcs;
    private int arc;
    private double errorSum;
    private int samples;
    private double meanError;
    private double manualStep;
    private boolean complete;

    public CentripetalPhase(TunerContext context, TuningValues values) {
        super(context);
        this.values = values;
        complete = values.centripetal > 0.0;
    }

    @Override
    protected String getPhaseName() {
        return "Centripetal";
    }

    @Override
    protected boolean manualTuneIsPossible() {
        return true;
    }

    @Override
    protected boolean autoTuneIsPossible() {
        return true;
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    protected void init() {
        complete = false;
        GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);
        Pose start = factory.pose(0, 0, 0);
        Pose end = factory.pose(48, 0, 0);
        arcs = new Path[]{
                factory.path(start, factory.arcPose(24, 24, 18), end)
                        .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild(),
                factory.path(end, factory.arcPose(24, -24, 18), start)
                        .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild()
        };

        double fullStrafeAcceleration = Math.max(values.strafeAcceleration / 0.95, 0.001);
        double seed = values.centripetal > 0.0 ? values.centripetal : 1.0 / fullStrafeAcceleration;
        double upper = Math.max(seed * 2.0, 2.0 / fullStrafeAcceleration);
        search = new BinarySearch(0.0, upper, upper / 64.0);
        values.centripetal = manualMode ? seed : search.getGuess();
        manualStep = Math.max(seed * 0.05, 0.00001);
        context.getFollower().setCentripetalTuning(values.centripetal);
        context.getFollower().setDriveControllerEnabled(false);
        startTrial();
    }

    private void startTrial() {
        context.getFollower().stop();
        context.getFollower().setPose(Pose.zero());
        arc = 0;
        errorSum = 0.0;
        samples = 0;
        context.getFollower().follow(arcs[0]);
    }

    private void sampleError() {
        PathSegment segment = arcs[arc].getParametricPath();
        Vector current = context.getFollower().getPose().getVec();
        double t = segment.getBestT(current);
        if (t <= 0.15 || t >= 0.85) {
            return;
        }

        Vector target = segment.getPosition(t);
        Vector normal = PathSegment.calculateArcNormal(segment.getFirstDerivative(t),
                segment.getSecondDerivative(t));
        double error = target.minus(current).dot(normal).getIn();
        if (Double.isFinite(error)) {
            errorSum += error;
            samples++;
        }
    }

    private boolean updateTrial() {
        if (context.getFollower().isBusy()) {
            sampleError();
            return false;
        }
        if (arc == 0) {
            arc = 1;
            context.getFollower().follow(arcs[1]);
            return false;
        }
        if (samples == 0) {
            throw new IllegalStateException("No centripetal error samples were recorded.");
        }
        meanError = errorSum / samples;
        return true;
    }

    private void finish() {
        context.getFollower().stop();
        context.getFollower().setDriveControllerEnabled(true);
        values.saveCentripetal(context);
        complete = true;
    }

    @Override
    protected void autoTuned() {
        if (!updateTrial()) {
            return;
        }
        if (Math.abs(meanError) <= ERROR_TARGET) {
            finish();
            return;
        }

        boolean keepSearching = search.updateGuess(meanError > 0.0);
        values.centripetal = search.getGuess();
        context.getFollower().setCentripetalTuning(values.centripetal);
        if (keepSearching) {
            startTrial();
        } else {
            finish();
        }
    }

    @Override
    protected void manualTuned() {
        boolean changed = false;
        if (opMode.gamepad1.dpadUpWasPressed()) {
            values.centripetal += manualStep;
            changed = true;
        }
        if (opMode.gamepad1.dpadDownWasPressed()) {
            values.centripetal = Math.max(0.0, values.centripetal - manualStep);
            changed = true;
        }
        if (opMode.gamepad1.dpadRightWasPressed()) {
            manualStep *= 2.0;
        }
        if (opMode.gamepad1.dpadLeftWasPressed()) {
            manualStep = Math.max(manualStep / 2.0, 0.000001);
        }

        if (changed) {
            context.getFollower().setCentripetalTuning(values.centripetal);
            startTrial();
        } else if (updateTrial()) {
            startTrial();
        }

        context.getTelemetry().addData("Centripetal", values.centripetal);
        context.getTelemetry().addData("Mean signed error", meanError);
        context.getTelemetry().addData("Step", manualStep);
        context.getTelemetry().addLine("Up/Down changes the gain.");
        context.getTelemetry().addLine("Left/Right changes the step.");
        context.getTelemetry().addLine("A accepts the value.");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            finish();
        }
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Centripetal", values.centripetal);
        context.getTelemetry().addData("Mean signed error", meanError);
    }
}
