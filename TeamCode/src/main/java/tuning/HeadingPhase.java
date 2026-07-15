package tuning;

import geometry.Angle;
import geometry.GeometryFactory;
import paths.movements.Turn;

public class HeadingPhase extends TuningPhase {
    private final TuningValues values;
    private PDSRoutine routine;
    private int selected;
    private double target = 90.0;
    private boolean complete;

    public HeadingPhase(TunerContext context, TuningValues values) {
        super(context);
        this.values = values;
        complete = values.heading.kP != 0.0;
    }

    @Override
    protected String getPhaseName() {
        return "Heading";
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
        selected = 0;
        if (manualMode) {
            context.getFollower().setHeadingTuning(values.heading);
            return;
        }
        routine = new PDSRoutine(context, TuningAxis.HEADING);
        routine.start();
    }

    @Override
    protected void autoTuned() {
        if (!routine.update(context)) {
            return;
        }
        values.heading = values.copy(routine.getCoefficients());
        context.getFollower().enableControllers();
        values.saveHeading(context);
        complete = true;
    }

    @Override
    protected void manualTuned() {
        if (opMode.gamepad1.dpadLeftWasPressed()) {
            selected = (selected + 2) % 3;
        }
        if (opMode.gamepad1.dpadRightWasPressed()) {
            selected = (selected + 1) % 3;
        }

        double direction = opMode.gamepad1.dpadUpWasPressed() ? 1.0 :
                opMode.gamepad1.dpadDownWasPressed() ? -1.0 : 0.0;
        if (direction != 0.0) {
            if (selected == 0) {
                values.heading.kP = Math.max(0.0, values.heading.kP + direction * 0.01);
            } else if (selected == 1) {
                values.heading.kD = Math.max(0.0, values.heading.kD + direction * 0.001);
            } else {
                values.heading.kS = Math.max(0.0, values.heading.kS + direction * 0.005);
            }
            context.getFollower().setHeadingTuning(values.heading);
        }

        if (opMode.gamepad1.xWasPressed() && !context.getFollower().isBusy()) {
            GeometryFactory factory = new GeometryFactory(context.getFollower());
            Turn turn = factory.turn(context.getFollower().getPose()).turnTo(Angle.fromDeg(target)).quickBuild();
            context.getFollower().follow(turn);
            target = -target;
        }

        context.getTelemetry().addData("Selected", selected == 0 ? "P" : selected == 1 ? "D" : "S");
        context.getTelemetry().addData("P", values.heading.kP);
        context.getTelemetry().addData("D", values.heading.kD);
        context.getTelemetry().addData("S", values.heading.kS);
        context.getTelemetry().addLine("Left/Right selects a value.");
        context.getTelemetry().addLine("Up/Down changes the value.");
        context.getTelemetry().addLine("X runs a test turn.");
        context.getTelemetry().addLine("A accepts the values.");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            values.saveHeading(context);
            complete = true;
        }
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading P", values.heading.kP);
        context.getTelemetry().addData("Heading D", values.heading.kD);
        context.getTelemetry().addData("Heading S", values.heading.kS);
    }
}
