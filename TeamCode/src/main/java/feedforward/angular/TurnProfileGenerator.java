package feedforward.angular;

import feedforward.FeedforwardLut;
import feedforward.MotionParameters;
import paths.movements.Turn;

/**
 * Generates a one-dimensional angular profile for point turns.
 * <p>
 * A turn does not move along a path, so the profile only fills {@code omega} and
 * {@code alpha}-related limits. The same LUT type is used so the follower can query turns and
 * paths through the same feedforward interface.
 */
public class TurnProfileGenerator {
    /** Maximum angular velocity allowed for the turn, in radians per second. */
    private double omega_max;
    /** Maximum angular acceleration allowed for the turn, in radians per second squared. */
    private double alpha_max;

    /**
     * Creates a turn profile generator with angular limits.
     *
     * @param omega_max maximum angular velocity
     * @param alpha_max maximum angular acceleration
     */
    public TurnProfileGenerator(double omega_max, double alpha_max) {
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
    }

    /**
     * Updates the angular limits without allocating a new generator.
     */
    public void setConstraints(double omega_max, double alpha_max) {
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
    }

    /**
     * Builds a trapezoidal-ish angular profile for the requested turn.
     * <p>
     * The pass structure mirrors the path generator: start with max velocity, sweep backward to
     * make sure the profile can stop, then sweep forward to make sure it can accelerate from rest.
     *
     * @param turn turn movement to profile
     * @param headingKS static friction feedforward for heading
     * @param angularKV velocity feedforward for heading
     * @param angularKA acceleration feedforward for heading
     * @return feedforward LUT containing angular velocity targets
     */
    public FeedforwardLut generate(Turn turn, double headingKS, double angularKV, double angularKA) {
        // Calculate the absolute angular distance of the turn (Assumes radians)
        double totalAngleRads = Math.abs(turn.getEndPose().getHeading().getShortestAngleTo(
                turn.getStartPose().getHeading()).getRad()
        );

        // Define structural bounds and target density (~2 degrees per step index)
        double targetRadPerStep = Math.toRadians(2.0);
        int minSteps = 15;
        int maxSteps = 200;

        // Calculate adaptive step size based on total sweep length
        int steps = (int) Math.ceil(totalAngleRads / targetRadPerStep) + 1;
        steps = Math.max(minSteps, Math.min(maxSteps, steps));

        double dTheta = totalAngleRads / (steps - 1);
        MotionParameters[] lut = new MotionParameters[steps];

        // Base pass: begin by assuming every angular sample can run at omega_max.
        for (int i = 0; i < steps; i++) {
            lut[i] = new MotionParameters();
            lut[i].setAngularVel(omega_max);
            lut[i].setTangentialVel(0.0); // No forward movement
        }

        // Backward pass: w^2 = w_next^2 + 2 * alpha * dTheta limits how fast we may enter
        // each remaining slice and still brake to zero by the end.
        lut[steps - 1].setAngularVel(0.0);
        for (int i = steps - 2; i >= 0; i--) {
            double nextW = lut[i + 1].getAngularVel();

            double maxReachableW = Math.sqrt((nextW * nextW) + (2.0 * alpha_max * dTheta));
            lut[i].setAngularVel(Math.min(lut[i].getAngularVel(), maxReachableW));
        }

        // Forward pass: grow angular velocity from rest, but reserve power for kS and kV.
        lut[0].setAngularVel(0.0);
        for (int i = 1; i < steps; i++) {
            double prevW = lut[i - 1].getAngularVel();

            // From normalized power 1 ~= kS + kV*w + kA*alpha, solve for remaining alpha.
            double dynamicAlpha = (1.0 - headingKS - (angularKV * prevW)) / angularKA;

            dynamicAlpha = Math.max(0.0, dynamicAlpha);
            double actualAlpha = Math.min(alpha_max, dynamicAlpha);

            double maxReachableW = Math.sqrt((prevW * prevW) + (2.0 * actualAlpha * dTheta));
            lut[i].setAngularVel(Math.min(lut[i].getAngularVel(), maxReachableW));
        }

        return new FeedforwardLut(lut);
    }
}
