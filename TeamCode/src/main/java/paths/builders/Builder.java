package paths.builders;

import geometry.Pose;

public class Builder {
    /**
     * Entry point for constructing continuous translational paths.
     */
    public static PathBuilder path(Pose... poses) {
        return new PathBuilder(poses);
    }

    /**
     * Entry point for constructing stationary point-turns.
     */
    public static TurnBuilder turn(Pose startPose) {
        return new TurnBuilder(startPose);
    }
}