package paths.builders;

import java.util.ArrayList;
import java.util.List;

import paths.callbacks.Callback;
import paths.movements.Turn;
import geometry.Angle;
import geometry.Pose;

/**
 * A builder class designed to construct a {@link Turn} fluently.
 * <p>
 * This handles stationary point-turns, allowing users to inject mechanical
 * callbacks at specific angles during the rotation.
 */
public class TurnBuilder {
    private final Pose startPose;
    private Angle targetHeading = null;

    private final List<Runnable> buildTasks = new ArrayList<>();

    /**
     * Initializes the TurnBuilder with the robot's starting state.
     * @param startPose The Pose of the robot before the turn begins.
     */
    protected TurnBuilder(Pose startPose) { this.startPose = startPose; }

    /**
     * Defines the target angle for the point turn.
     * @param targetHeading The Angle to rotate to.
     * @return The current TurnBuilder instance.
     */
    public TurnBuilder turnTo(Angle targetHeading) {
        this.targetHeading = targetHeading;
        return this;
    }

    /**
     * Attaches an executable callback to trigger when the robot passes a specific angle during the turn.
     *
     * @param angle The angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current TurnBuilder instance for method chaining.
     */
    public TurnBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            if (targetHeading == null) return;
            Turn finalTurn = new Turn(startPose, targetHeading);
            Angle startRad = finalTurn.getStartPose().getHeading();
            Angle endRad = finalTurn.getEndPose().getHeading();

            double totalDiff = startRad.getShortestAngleTo(endRad).getRad();
            double targetDiff = startRad.getShortestAngleTo(angle).getRad();

            if (Math.abs(totalDiff) < 1e-6) {
                if (Math.abs(targetDiff) > 1e-6) {
                    throw new IllegalArgumentException("Callback out of bounds: The turn has no rotational distance.");
                }
            } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                throw new IllegalArgumentException("Angular callback is outside the sweep range of this turn.");
            }
        });

        return this;
    }

    /**
     * Compiles the turn, verifies callback bounds, and returns the executable Turn movement.
     * @return The fully constructed {@link Turn}.\
     */
    public Turn build() {
        if (targetHeading == null) {
            throw new IllegalStateException("Cannot build Turn: No target heading was specified! Use .turnTo().");
        }

        Turn finalTurn = new Turn(startPose, targetHeading);

        for (Runnable task : buildTasks) {
            task.run();
        }

        for (Runnable task : buildTasks) {
            finalTurn.addCallback(new Callback(targetHeading, task));
        }

        return finalTurn;
    }
}