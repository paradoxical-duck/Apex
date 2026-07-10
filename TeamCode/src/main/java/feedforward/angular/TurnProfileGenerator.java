package feedforward.angular;

import core.FollowerConstants;
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
    private static final double EPSILON = 1e-9;
    private static final int POWER_SEARCH_ITERATIONS = 40;

    /** Maximum angular velocity allowed for the turn, in radians per second. */
    private double omega_max;
    /** Maximum angular acceleration allowed for the turn, in radians per second squared. */
    private double alpha_max;
    private final double angularKV;
    private final double angularKA;
    private final double headingKS;
    /**
     * Creates a turn profile generator with angular limits.
     *
     * @param omega_max maximum angular velocity
     * @param alpha_max maximum angular acceleration
     */
    public TurnProfileGenerator(double omega_max, double alpha_max) {
        this(omega_max, alpha_max, new FollowerConstants());
    }

    /** Creates a generator from one explicit follower-configuration source. */
    public TurnProfileGenerator(double omega_max, double alpha_max, FollowerConstants config) {
        this(omega_max, alpha_max, config.angularKV, config.angularKA,
                config.headingCoeffs.kS);
    }

    /**
     * Creates a turn profile generator with drivetrain power-model coefficients.
     *
     * @param angularKV normalized power per radian/second
     * @param angularKA normalized power per radian/second-squared
     * @param headingKS normalized static-friction power
     */
    public TurnProfileGenerator(double omega_max, double alpha_max,
                                double angularKV, double angularKA, double headingKS) {
        validateConstraints(omega_max, alpha_max);
        validatePowerCoefficients(angularKV, angularKA, headingKS);
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.headingKS = headingKS;
    }

    /**
     * Updates the angular limits without allocating a new generator.
     */
    public void setConstraints(double omega_max, double alpha_max) {
        validateConstraints(omega_max, alpha_max);
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
     * @return feedforward LUT containing angular velocity targets
     */
    public FeedforwardLut generate(Turn turn) {
        double signedTurn = turn.getStartPose().getHeading()
                .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
        double direction = Math.signum(signedTurn);
        double totalAngleRads = Math.abs(signedTurn);

        if (totalAngleRads < 1e-9) {
            MotionParameters stationary = new MotionParameters();
            stationary.setDistAlongCurve(0.0);
            return new FeedforwardLut(new MotionParameters[]{stationary});
        }

        // Define structural bounds and target density (~2 degrees per step index)
        double targetRadPerStep = Math.toRadians(2.0);
        int minSteps = 15;
        int maxSteps = 200;

        // Calculate adaptive step size based on total sweep length
        int steps = (int) Math.ceil(totalAngleRads / targetRadPerStep) + 1;
        steps = Math.max(minSteps, Math.min(maxSteps, steps));

        double dTheta = totalAngleRads / (steps - 1);
        MotionParameters[] lut = new MotionParameters[steps];
        double profileVelocityLimit = getPowerLimitedVelocity(omega_max);

        // Base pass: cap speed by both the configured kinematic limit and back-EMF power.
        for (int i = 0; i < steps; i++) {
            lut[i] = new MotionParameters();
            lut[i].setAngularVel(profileVelocityLimit);
            lut[i].setTangentialVel(0.0); // No forward movement
            lut[i].setDistAlongCurve(i * dTheta);
        }

        // Backward pass: w^2 = w_next^2 + 2 * alpha * dTheta limits how fast we may enter
        // each remaining slice and still brake to zero by the end.
        lut[steps - 1].setAngularVel(0.0);
        for (int i = steps - 2; i >= 0; i--) {
            double nextW = lut[i + 1].getAngularVel();
            double maxReachableW = findMaxPreviousVelocity(
                    nextW, dTheta, profileVelocityLimit);
            lut[i].setAngularVel(Math.min(lut[i].getAngularVel(), maxReachableW));
        }

        // Forward pass: reserve voltage for back EMF and static friction before accelerating.
        lut[0].setAngularVel(0.0);
        for (int i = 1; i < steps; i++) {
            double prevW = lut[i - 1].getAngularVel();
            double availableAcceleration = getMaxForwardAcceleration(prevW);
            double maxReachableW =
                    Math.sqrt((prevW * prevW) + (2.0 * availableAcceleration * dTheta));
            lut[i].setAngularVel(Math.min(lut[i].getAngularVel(), maxReachableW));
        }

        // Convert the scalar profile into signed angular states. Acceleration belongs to the
        // segment beginning at each sample so the first row can command the turn from rest.
        for (int i = 0; i < steps - 1; i++) {
            double currentW = lut[i].getAngularVel();
            double nextW = lut[i + 1].getAngularVel();
            double acceleration = ((nextW * nextW) - (currentW * currentW)) /
                    (2.0 * dTheta);
            lut[i].setAngularVel(direction * currentW);
            lut[i].setAngularAccel(direction * acceleration);
        }
        lut[steps - 1].setAngularVel(0.0);
        lut[steps - 1].setAngularAccel(0.0);

        return new FeedforwardLut(lut);
    }

    private static void validateConstraints(double omegaMax, double alphaMax) {
        if (!Double.isFinite(omegaMax) || omegaMax <= 0.0) {
            throw new IllegalArgumentException("Maximum angular velocity must be positive.");
        }
        if (!Double.isFinite(alphaMax) || alphaMax <= 0.0) {
            throw new IllegalArgumentException("Maximum angular acceleration must be positive.");
        }
    }

    private static void validatePowerCoefficients(double angularKV, double angularKA,
                                                  double headingKS) {
        if (!Double.isFinite(angularKV) || angularKV < 0.0 ||
                !Double.isFinite(angularKA) || angularKA < 0.0 ||
                !Double.isFinite(headingKS) || headingKS < 0.0 || headingKS >= 1.0) {
            throw new IllegalArgumentException(
                    "Angular kV/kA must be nonnegative and heading kS must be in [0, 1)."
            );
        }
    }

    private double getPowerLimitedVelocity(double configuredLimit) {
        if (angularKV <= EPSILON) {
            return configuredLimit;
        }
        return Math.min(configuredLimit, Math.max(0.0, (1.0 - headingKS) / angularKV));
    }

    /** Solves kV*w + kA*a + kS <= 1 for positive acceleration. */
    private double getMaxForwardAcceleration(double angularVelocity) {
        if (angularKA <= EPSILON) {
            return alpha_max;
        }
        double powerLimited =
                (1.0 - headingKS - (angularKV * angularVelocity)) / angularKA;
        return Math.min(alpha_max, Math.max(0.0, powerLimited));
    }

    /**
     * Solves |kV*w - kA*d + kS| <= 1 for braking magnitude d.
     */
    private double getMaxBrakingAcceleration(double angularVelocity) {
        if (angularKA <= EPSILON) {
            return alpha_max;
        }
        double powerLimited =
                (1.0 + headingKS + (angularKV * angularVelocity)) / angularKA;
        return Math.min(alpha_max, Math.max(0.0, powerLimited));
    }

    private double findMaxPreviousVelocity(double nextVelocity, double displacement,
                                           double velocityLimit) {
        double low = nextVelocity;
        double high = Math.max(nextVelocity, velocityLimit);

        for (int i = 0; i < POWER_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            double requiredDeceleration =
                    ((candidate * candidate) - (nextVelocity * nextVelocity)) /
                            (2.0 * displacement);
            if (requiredDeceleration <= getMaxBrakingAcceleration(candidate) + EPSILON) {
                low = candidate;
            } else {
                high = candidate;
            }
        }
        return low;
    }
}
