package paths;

import paths.builders.PathBuilder;
import paths.builders.TurnBuilder;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;
import paths.heading.InterpolationStyle;

import geometry.Angle;
import geometry.Pose;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

public class FullAPIShowcase {
    private final DistUnit distUnit = DistUnit.IN;
    private final AngleUnit angleUnit = AngleUnit.DEG;
    public PoseFactory pose = new PoseFactory(distUnit, angleUnit);
    private Pose startPose;

    // Routine movements stored in sequence
    public Path smoothBlendPath;
    public Path constStartPath;
    public Path constEndPath;
    public Turn callbackTurn;
    public Path tangentForwardPath;
    public Path tangentOptimalPath;
    public Path tangentCustomPath;
    public Path lambdaSpinPath;

    public FullAPIShowcase(PoseFactory.Mirror mirror) {
        pose.setMirror(mirror);
        startPose = Pose.Common.CENTER.get(); // (0, 0, 0)
        buildRoutine();
    }

    // region Dummy Actions
    public void startIntake() { /* Starts intake motors */ }
    public void deployOuttake() { /* Extends slides */ }
    public void dropElement() { /* Opens claw */ }
    public void finishRoutine() { /* Plays an LED animation */ }
    // endregion

    /**
     * A comprehensive showcase of every InterpolationStyle in the API.
     */
    private void buildRoutine() {

        // 1. SMOOTH_START_TO_END
        // Blends the heading linearly from the start pose to the end pose.
        smoothBlendPath = new PathBuilder()
                .addControlPoints(
                        startPose,
                        pose.of(15, 0),
                        pose.of(25, 15, 90) // Target heading is 90 degrees
                )
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                .build();

        // ---------------------------------------------------------

        // 2. CONSTANT_START_HEADING
        // The robot locks its orientation to whatever heading it had when this path started.
        // Great for pure strafing. It will stay locked at 90 degrees from the previous path.
        constStartPath = new PathBuilder()
                .addControlPoints(
                        smoothBlendPath.getEndPose(),
                        pose.of(25, 40),
                        pose.of(45, 40, 180) // APEX WARNING: The 180 target is ignored due to CONSTANT_START_HEADING
                )
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .addDistanceCallback(0.5, this::startIntake)
                .build();

        // ---------------------------------------------------------

        // 3. CONSTANT_END_HEADING
        // The robot immediately targets the final heading of the segment and holds it for the whole curve.
        constEndPath = new PathBuilder()
                .addControlPoints(
                        constStartPath.getEndPose(),
                        pose.of(60, 20),
                        pose.of(60, 0, 270) // The robot will immediately pivot to face 270 while driving
                )
                .interpolateWith(InterpolationStyle.CONSTANT_END_HEADING)
                .build();

        // ---------------------------------------------------------

        // 4. POINT TURN WITH ANGULAR CALLBACK
        // Spins in place from 270 degrees up to 360/0 degrees.
        callbackTurn = new TurnBuilder(constEndPath.getEndPose())
                .turnTo(Angle.fromDeg(0))

                // Triggers exactly when the robot sweeps past the 315-degree mark during the spin
                .addAngularCallback(Angle.fromDeg(315), this::deployOuttake)
                .build();

        // ---------------------------------------------------------

        // 5. TANGENT_FORWARD
        // The robot strictly faces the forward direction of travel along the path (like a car).
        tangentForwardPath = new PathBuilder()
                .addControlPoints(
                        callbackTurn.getEndPose(),
                        pose.of(80, 0),
                        pose.of(100, 20, 90) // Target heading ignored; overridden by path tangent
                )
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
                .build();

        // ---------------------------------------------------------

        // 6. TANGENT_OPTIMAL
        // Points either forward OR backward along the path depending on which requires less physical rotation.
        tangentOptimalPath = new PathBuilder()
                .addControlPoints(
                        tangentForwardPath.getEndPose(),
                        pose.of(120, 20),
                        pose.arcPoseOf(120, 0, 10), // Dynamically fillets this sharp corner
                        pose.of(100, -20, 0)
                )
                .interpolateWith(InterpolationStyle.TANGENT_OPTIMAL)
                .addDistanceCallback(0.8, this::dropElement)
                .build();

        // ---------------------------------------------------------

        // 7. TANGENT_CUSTOM
        // Follows the path tangent, but allows for a custom, fixed angular offset.
        tangentCustomPath = new PathBuilder()
                .addControlPoints(
                        tangentOptimalPath.getEndPose(),
                        pose.of(80, -20),
                        pose.of(60, 0, 0)
                )
                .interpolateWith(InterpolationStyle.TANGENT_CUSTOM, Angle.fromDeg(90))
                // NOTE: Depending on your API implementation, you may need a method here to pass the custom offset angle!
                .build();

        // ---------------------------------------------------------

        // 8. CUSTOM_DIST_FUNCTION (LAMBDA OVERRIDE)
        // Complete mathematical control over the heading.
        // This calculates a 360-degree tornado spin scaling with path progression (s).
        lambdaSpinPath = new PathBuilder()
                .addControlPoints(
                        tangentCustomPath.getEndPose(),
                        pose.of(30, 0),
                        pose.of(0, 0, 0)
                )
                // (s) represents the physical percentage along the path [0.0 to 1.0]
                .interpolateWith(s -> Angle.fromDeg(s * 360.0))
                .addDistanceCallback(1.0, this::finishRoutine)
                .build();
    }

    /**
     * Helper to retrieve the full, pre-compiled routine for the Follower's state machine.
     */
    public FollowerMovement[] getAutoRoutine() {
        return new FollowerMovement[] {
                smoothBlendPath,
                constStartPath,
                constEndPath,
                callbackTurn,
                tangentForwardPath,
                tangentOptimalPath,
                tangentCustomPath,
                lambdaSpinPath
        };
    }
}