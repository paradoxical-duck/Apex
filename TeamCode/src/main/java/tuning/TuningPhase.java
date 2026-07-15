package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

enum TuningState {
    SELECT_MODE,
    TUNING,
    RESULTS
}

public abstract class TuningPhase {
    protected final TunerContext context;
    protected LinearOpMode opMode;
    protected boolean manualMode;
    private TuningState state = TuningState.SELECT_MODE;

    protected TuningPhase(TunerContext context) {
        this.context = context;
    }

    public void run(LinearOpMode opMode) {
        this.opMode = opMode;
        manualMode = false;
        state = TuningState.SELECT_MODE;

        while (opMode.opModeIsActive()) {
            switch (state) {
                case SELECT_MODE:
                    showModeSelector();
                    if (manualTuneIsPossible() && autoTuneIsPossible() && opMode.gamepad1.bWasPressed()) {
                        manualMode = !manualMode;
                    }
                    if (opMode.gamepad1.aWasPressed()) {
                        init();
                        state = TuningState.TUNING;
                    }
                    break;
                case TUNING:
                    context.getFollower().update();
                    if (manualMode) {
                        manualTune();
                    } else {
                        autoTune();
                    }
                    if (isComplete()) {
                        context.getFollower().stop();
                        state = TuningState.RESULTS;
                    }
                    break;
                case RESULTS:
                    showResults();
                    if (opMode.gamepad1.bWasPressed()) {
                        return;
                    }
                    break;
            }
            opMode.idle();
        }

        context.getFollower().stop();
    }

    private void showModeSelector() {
        context.getTelemetry().addLine(getPhaseName() + " phase initialized");
        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            context.getTelemetry().addLine("Press B to toggle between automatic and manual tuning.");
            context.getTelemetry().addData("Selected Mode", manualMode ? "Manual" : "Automatic");
        } else {
            manualMode = manualTuneIsPossible();
            context.getTelemetry().addData("Selected Mode", manualMode ? "Manual" : "Automatic");
        }
        context.getTelemetry().addLine("Press A to run this phase.");
        context.getTelemetry().update();
    }

    private void showResults() {
        context.getTelemetry().addLine(getPhaseName() + " phase complete with results:");
        reportResults();
        context.getTelemetry().addLine("Press B to continue.");
        context.getTelemetry().update();
    }

    protected abstract String getPhaseName();

    protected abstract boolean manualTuneIsPossible();

    protected abstract boolean autoTuneIsPossible();

    public abstract boolean isComplete();

    protected abstract void init();

    protected abstract void manualTune();

    protected abstract void autoTune();

    protected abstract void reportResults();
}
