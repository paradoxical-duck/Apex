package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import geometry.Pose;

/**
 * Localizer verification before commencing tuning
 * @author Sohum Arora 22985 Paraducks
 */
public class PreflightPhase extends TuningPhase {
    private static final double TEST_POWER = 0.22;
    private static final double TEST_SECONDS = 0.45;
    private static final double MINIMUM_DELTA = 0.05;

    private final ElapsedTime timer = new ElapsedTime();
    private CharacterizationAxis[] axes;
    private int index;
    private Pose start;
    private String failure;
    private boolean manualPreflight;

    public PreflightPhase(TunerContext context) { super(context); }

    @Override protected String getPhaseName() { return "Hardware / Localizer Preflight"; }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override
    protected void init() {
        axes = context.getFollower().getDrivetrain().isHolonomic()
                ? new CharacterizationAxis[]{CharacterizationAxis.FORWARD,
                CharacterizationAxis.STRAFE, CharacterizationAxis.ANGULAR}
                : new CharacterizationAxis[]{CharacterizationAxis.FORWARD,
                CharacterizationAxis.ANGULAR};
        index = 0;
        failure = null;
        manualPreflight = isManualMode();
        context.getFollower().disableControllers();
        context.getFollower().stop();
        context.getFollower().setPose(Pose.zero());
        start = context.getFollower().getPose();
        timer.reset();
    }

    @Override
    protected boolean manualUpdate(boolean aWasPressed, boolean bWasPressed) {
        Gamepad gamepad = context.getGamepad();
        if (bWasPressed) {
            context.getFollower().getDrivetrain().stop();
            return true;
        }
        if (aWasPressed) context.getFollower().setPose(Pose.zero());

        double forward = gamepad.dpad_up ? TEST_POWER :
                (gamepad.dpad_down ? -TEST_POWER : 0.0);
        double strafe = gamepad.dpad_left ? TEST_POWER :
                (gamepad.dpad_right ? -TEST_POWER : 0.0);
        double turn = gamepad.left_bumper ? TEST_POWER :
                (gamepad.right_bumper ? -TEST_POWER : 0.0);
        if (!context.getFollower().getDrivetrain().isHolonomic()) strafe = 0.0;
        context.getFollower().getDrivetrain().moveWithVectors(forward, strafe, turn);

        context.getTelemetry().addLine("MANUAL HARDWARE / LOCALIZER PREFLIGHT");
        context.getTelemetry().addLine("D-pad Up/Down - forward/backward");
        context.getTelemetry().addLine("D-pad Left/Right - left/right strafe");
        context.getTelemetry().addLine("Bumpers - counterclockwise/clockwise turn");
        context.getTelemetry().addLine("A - reset pose, B - finish");
        context.getTelemetry().addData("Pose", context.getFollower().getPose());
        context.getTelemetry().addData("Velocity", context.getFollower().getVelocity());
        return false;
    }

    @Override
    protected boolean automaticUpdate() {
        CharacterizationAxis axis = axes[index];
        context.driveOpenLoop(axis, TEST_POWER);
        context.getTelemetry().addData("Testing", axis.displayName());

        if (timer.seconds() < TEST_SECONDS) return false;

        context.getFollower().getDrivetrain().stop();
        Pose delta = context.getFollower().getPose().minus(start);
        double expected = axis.position(delta);
        double largestOther = largestOtherAxis(axis, delta);
        if (expected <= MINIMUM_DELTA) {
            failure = axis.displayName() + " command did not produce a positive " +
                    axis.displayName().toLowerCase() + " localizer delta. Check motor/encoder " +
                    "directions and the X-forward/Y-left convention.";
            return true;
        }
        if (largestOther > Math.abs(expected) * 0.75) {
            failure = axis.displayName() + " command was reported primarily on another axis. " +
                    "Fix localizer axis assignment before tuning.";
            return true;
        }

        index++;
        if (index >= axes.length) return true;
        context.getFollower().setPose(Pose.zero());
        start = context.getFollower().getPose();
        timer.reset();
        return false;
    }

    private double largestOtherAxis(CharacterizationAxis expected, Pose delta) {
        double largest = 0.0;
        for (CharacterizationAxis candidate : CharacterizationAxis.values()) {
            if (candidate == expected) continue;
            largest = Math.max(largest, Math.abs(candidate.position(delta)));
        }
        return largest;
    }

    @Override
    protected void reportResults() {
        if (manualPreflight) {
            context.getTelemetry().addLine("Manual preflight finished. Confirm every commanded " +
                    "axis moved in the matching positive and negative pose direction.");
            return;
        }
        if (failure == null) {
            context.getTelemetry().addLine("PASS: drivetrain commands and localizer axes agree.");
        } else {
            context.getTelemetry().addLine("FAIL: " + failure);
            context.getTelemetry().addLine("Do not run characterization until this is fixed.");
        }
    }
}
