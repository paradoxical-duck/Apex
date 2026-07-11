package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Pose;

/**
 * Bidirectional ramp and step characterization for one drivetrain axis. This replaces the unsafe
 * single-step Ziegler-Nichols routine and identifies kS, kV, and kA from many samples.
 */
public abstract class AxisCharacterizationPhase extends TuningPhase {
    private static final double RAMP_SECONDS = 3.0;
    private static final double RAMP_MAX_POWER = 0.70;
    private static final double STEP_SECONDS = 1.25;
    private static final double STEP_POWER = 0.75;
    private static final double SETTLE_SECONDS = 0.75;

    private enum TestState {
        RAMP_POSITIVE, SETTLE_1, RAMP_NEGATIVE, SETTLE_2,
        STEP_POSITIVE, SETTLE_3, STEP_NEGATIVE, FINISHED
    }

    protected final CharacterizationAxis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private final FeedforwardFit fit = new FeedforwardFit();
    private TestState state;
    private long lastSampleNanos;
    private double lastVelocity;
    private double filteredAcceleration;
    private double accumulatedTravel;
    private FeedforwardFit.Result result;
    private double safeVelocity;
    private double safeAcceleration;
    private String failure;
    private ManualParameter[] manualParameters;
    private int selectedManualParameter;
    private PDSController manualPositionController;
    private boolean manualDriving;
    private double manualOutput;

    protected AxisCharacterizationPhase(TunerContext context, CharacterizationAxis axis) {
        super(context);
        this.axis = axis;
    }

    @Override protected String getPhaseName() { return axis.displayName() + " Characterization"; }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualUpdate() { return false; }

    @Override
    protected void init() {
        context.getFollower().disableControllers();
        context.getFollower().stop();
        context.getFollower().setPose(Pose.zero());
        if (isManualMode()) {
            manualParameters = createManualParameters();
            selectedManualParameter = 0;
            manualPositionController = new PDSController(manualPositionCoefficients());
            if (axis == CharacterizationAxis.ANGULAR) {
                manualPositionController.setAngularController();
            }
            manualDriving = false;
            manualOutput = 0.0;
            return;
        }
        state = TestState.RAMP_POSITIVE;
        lastSampleNanos = -1;
        lastVelocity = 0.0;
        filteredAcceleration = 0.0;
        accumulatedTravel = 0.0;
        timer.reset();
    }

    @Override
    protected boolean manualUpdate(boolean aWasPressed, boolean bWasPressed) {
        Gamepad gamepad = context.getGamepad();
        if (bWasPressed) {
            context.getFollower().getDrivetrain().stop();
            return true;
        }

        if (gamepad.dpadLeftWasPressed()) {
            selectedManualParameter = (selectedManualParameter - 1 + manualParameters.length) %
                    manualParameters.length;
        } else if (gamepad.dpadRightWasPressed()) {
            selectedManualParameter = (selectedManualParameter + 1) % manualParameters.length;
        }

        double adjustmentScale = gamepad.right_bumper ? 10.0 :
                (gamepad.left_bumper ? 0.1 : 1.0);
        if (gamepad.dpadUpWasPressed()) {
            manualParameters[selectedManualParameter].adjust(1, adjustmentScale);
        } else if (gamepad.dpadDownWasPressed()) {
            manualParameters[selectedManualParameter].adjust(-1, adjustmentScale);
        }

        if (aWasPressed) {
            context.getFollower().getDrivetrain().stop();
            context.getFollower().setPose(Pose.zero());
            manualPositionController.reset();
        }

        double currentPosition = axis.position(context.getFollower().getPose());
        double currentVelocity = axis.velocity(context.getFollower().getVelocity());
        double triggerCommand = gamepad.right_trigger - gamepad.left_trigger;
        boolean positionTest = gamepad.x || gamepad.y;
        boolean velocityTest = Math.abs(triggerCommand) > 0.05;

        if (velocityTest) {
            double velocityLimit = Math.max(manualMinimumTestVelocity(), manualVelocityLimit());
            double targetVelocity = triggerCommand * velocityLimit * 0.35;
            manualOutput = (Math.signum(targetVelocity) * manualPositionCoefficients().kS) +
                    (manualKV() * targetVelocity) +
                    (manualVelocityFeedbackGain() * (targetVelocity - currentVelocity));
            context.driveOpenLoop(axis, Range.clip(manualOutput, -0.70, 0.70));
            manualDriving = true;
        } else if (positionTest) {
            double target = gamepad.x ? manualPositionTarget() : -manualPositionTarget();
            if (!manualDriving) manualPositionController.reset();
            manualOutput = manualPositionController.calculateFromError(target - currentPosition);
            context.driveOpenLoop(axis, Range.clip(manualOutput, -0.70, 0.70));
            manualDriving = true;
        } else {
            context.getFollower().getDrivetrain().stop();
            if (manualDriving) manualPositionController.reset();
            manualDriving = false;
            manualOutput = 0.0;
        }

        context.getTelemetry().addLine("MANUAL " + axis.displayName().toUpperCase() + " TUNING");
        context.getTelemetry().addLine("D-pad Left/Right - select value");
        context.getTelemetry().addLine("D-pad Up/Down - increase/decrease");
        context.getTelemetry().addLine("Left bumper = fine, Right bumper = coarse");
        context.getTelemetry().addLine("Hold X/Y - test positive/negative position target");
        context.getTelemetry().addLine("Triggers - test positive/negative velocity");
        context.getTelemetry().addLine("A - reset pose, B - save and finish");
        for (int i = 0; i < manualParameters.length; i++) {
            String marker = i == selectedManualParameter ? "> " : "  ";
            context.getTelemetry().addData(marker + manualParameters[i].getName(),
                    "%.6f", manualParameters[i].get());
        }
        context.getTelemetry().addData("Position", "%.4f", currentPosition);
        context.getTelemetry().addData("Velocity", "%.4f", currentVelocity);
        context.getTelemetry().addData("Output", "%.4f", manualOutput);
        return false;
    }

    @Override
    protected boolean automaticUpdate() {
        if (state == TestState.FINISHED) return finishFit();

        double elapsed = timer.seconds();
        double command = commandForState(elapsed);
        context.driveOpenLoop(axis, command);
        sample(command);

        context.getTelemetry().addData("Axis", axis.displayName());
        context.getTelemetry().addData("Test", state);
        context.getTelemetry().addData("Command", "%.3f", command);
        context.getTelemetry().addData("Samples", fit.size());
        context.getTelemetry().addData("Travel", "%.2f", accumulatedTravel);

        double stateDuration = durationForState();
        boolean travelLimitReached = accumulatedTravel >= axis.maximumTestTravel();
        if (elapsed >= stateDuration || (travelLimitReached && isPoweredState())) {
            advanceState();
        }
        return false;
    }

    private void sample(double command) {
        long now = System.nanoTime();
        double velocity = axis.velocity(context.getFollower().getVelocity());
        if (lastSampleNanos != -1) {
            double dt = (now - lastSampleNanos) / 1e9;
            if (dt >= 0.002 && dt <= 0.20) {
                double rawAcceleration = (velocity - lastVelocity) / dt;
                filteredAcceleration = (0.82 * filteredAcceleration) + (0.18 * rawAcceleration);
                accumulatedTravel += Math.abs(velocity) * dt;
                if (isPoweredState()) {
                    fit.add(command, velocity, filteredAcceleration,
                            axis.minimumSampleVelocity());
                }
            }
        }
        lastVelocity = velocity;
        lastSampleNanos = now;
    }

    private double commandForState(double elapsed) {
        switch (state) {
            case RAMP_POSITIVE: return RAMP_MAX_POWER * Math.min(1.0, elapsed / RAMP_SECONDS);
            case RAMP_NEGATIVE: return -RAMP_MAX_POWER * Math.min(1.0, elapsed / RAMP_SECONDS);
            case STEP_POSITIVE: return STEP_POWER;
            case STEP_NEGATIVE: return -STEP_POWER;
            default: return 0.0;
        }
    }

    private boolean isPoweredState() {
        return state == TestState.RAMP_POSITIVE || state == TestState.RAMP_NEGATIVE ||
                state == TestState.STEP_POSITIVE || state == TestState.STEP_NEGATIVE;
    }

    private double durationForState() {
        switch (state) {
            case RAMP_POSITIVE:
            case RAMP_NEGATIVE: return RAMP_SECONDS;
            case STEP_POSITIVE:
            case STEP_NEGATIVE: return STEP_SECONDS;
            default: return SETTLE_SECONDS;
        }
    }

    private void advanceState() {
        context.getFollower().getDrivetrain().stop();
        state = TestState.values()[state.ordinal() + 1];
        timer.reset();
        accumulatedTravel = 0.0;
        filteredAcceleration = 0.0;
        lastSampleNanos = -1;
        lastVelocity = 0.0;
        if (state != TestState.FINISHED) context.getFollower().setPose(Pose.zero());
    }

    private boolean finishFit() {
        context.getFollower().getDrivetrain().stop();
        result = fit.solve();
        safeVelocity = fit.percentileAbsoluteVelocity(0.90) * 0.90;
        safeAcceleration = fit.percentileAbsoluteAcceleration(0.90) * 0.80;
        if (!result.isUsable()) {
            failure = String.format(
                    "Rejected fit: samples=%d, R^2=%.3f, kS=%.5f, kV=%.5f, kA=%.5f",
                    result.sampleCount, result.rSquared, result.kS, result.kV, result.kA);
        } else if (safeVelocity <= 0.0 || safeAcceleration <= 0.0) {
            failure = "Rejected fit: the localizer did not report usable velocity/acceleration.";
        } else {
            applyResult(result, safeVelocity, safeAcceleration);
        }
        return true;
    }

    protected abstract void applyResult(FeedforwardFit.Result result,
                                        double safeVelocity, double safeAcceleration);

    protected abstract ManualParameter[] createManualParameters();

    protected abstract PDSCoefficients manualPositionCoefficients();

    protected abstract double manualKV();

    protected abstract double manualVelocityFeedbackGain();

    protected abstract double manualVelocityLimit();

    protected double manualPositionTarget() {
        return axis == CharacterizationAxis.ANGULAR ? Math.toRadians(90.0) : 24.0;
    }

    protected double manualMinimumTestVelocity() {
        return axis == CharacterizationAxis.ANGULAR ? 1.0 : 12.0;
    }

    protected PDSCoefficients positionGains(FeedforwardFit.Result result) {
        double settlingSeconds = axis == CharacterizationAxis.ANGULAR ? 0.75 : 1.00;
        return result.positionGains(settlingSeconds);
    }

    @Override
    protected void reportResults() {
        if (isManualMode()) {
            context.getTelemetry().addLine("Manual values saved:");
            for (ManualParameter parameter : manualParameters) {
                context.getTelemetry().addData(parameter.getName(), "%.6f", parameter.get());
            }
            return;
        }
        if (failure != null) {
            context.getTelemetry().addLine(failure);
            context.getTelemetry().addLine("Nothing was saved. Check localizer axes and rerun.");
            return;
        }
        context.getTelemetry().addData("Samples", result.sampleCount);
        context.getTelemetry().addData("R squared", "%.4f", result.rSquared);
        context.getTelemetry().addData("kS", "%.6f", result.kS);
        context.getTelemetry().addData("kV", "%.6f", result.kV);
        context.getTelemetry().addData("kA", "%.6f", result.kA);
        context.getTelemetry().addData("Safe velocity", "%.3f", safeVelocity);
        context.getTelemetry().addData("Safe acceleration", "%.3f", safeAcceleration);
    }
}
