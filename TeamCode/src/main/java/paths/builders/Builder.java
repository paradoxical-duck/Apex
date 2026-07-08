package paths.builders;

import geometry.Pose;

/**
 * @author Sohum Arora 22985 Paraducks
 */
public class Builder {
    /**
     * Entry point for constructing holonomic paths.
     */
    public static HolonomicPathBuilder holonomicPath(Pose... poses) {
        return new HolonomicPathBuilder(poses);
    }

    /**
     * Entry point for constructing tank path
     */
    public static TankPathBuilder tankPath(Pose... poses) {
        return new TankPathBuilder(poses);
    }

    /**
     * Entry point for constructing stationary point-turns.
     */
    public static TurnBuilder turn(Pose startPose) {
        return new TurnBuilder(startPose);
    }
}