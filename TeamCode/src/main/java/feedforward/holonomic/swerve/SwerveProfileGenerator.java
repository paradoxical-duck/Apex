package feedforward.holonomic.swerve;

import core.FollowerConstants;
import feedforward.BaseProfileGenerator;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/**
 * Generates time-optimal motion profiles for Swerve drivetrains by evaluating
 * continuous voltage saturation constraints algebraically.
 */
public class SwerveProfileGenerator extends BaseProfileGenerator {
    private static final double EPSILON = 1e-6;
    private static final int VELOCITY_SEARCH_ITERATIONS = 10;

    private final FollowerConstants config;

    public SwerveProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
    }

    // region Base Pass (Velocity Ceiling)

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point, Path path,
                                                    double maxAngVel, double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalVel = config.forwardVelocityLimit.getIn();
        double effectiveAngVelLimit = Math.min(config.angularVelocityLimit.getRad(), maxAngVel);
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);

        double min_v = 0.0;
        double max_v = maxPhysicalVel;

        for (int i = 0; i < VELOCITY_SEARCH_ITERATIONS; i++) {
            double mid_v = (min_v + max_v) / 2.0;
            double omega = fPrime * mid_v;
            double alpha = fDoublePrime * mid_v * mid_v;
            boolean violatesAngularLimit =
                    Math.abs(omega) > effectiveAngVelLimit + EPSILON ||
                            Math.abs(alpha) > effectiveAngAccelLimit + EPSILON;

            if (violatesAngularLimit || calculatePowerUtilization(mid_v, 0.0, path, point) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    // endregion

    // region Integration Passes (Acceleration / Deceleration)

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point,
                                              Path path, double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, true);
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point,
                                           Path path, double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, false);
    }

    // endregion

    // region Utility and Evaluation

    /**
     * Calculates the theoretical total motor power required to execute a given kinematic state.
     */
    protected double calculatePowerUtilization(double vel, double accel, Path path, PathPoint point) {
        EvaluationResult result = new EvaluationResult();
        evaluateState(path, point, vel, accel, result);
        return result.totalPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        evaluateState(path, current, v, a_t, outResult);
    }

    private void evaluateState(Path path, PathPoint point, double vel, double accel,
                               EvaluationResult outResult) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

        double tanPow = (vel * config.translationalKV)
                + (accel * config.translationalKA)
                + signedStatic(vel, accel, config.translationalCoeffs.kS);

        double normPow = (vel * vel * kappa) * config.Kcentripetal;

        double omega = fPrime * vel;
        double alpha = (fDoublePrime * (vel * vel)) + (fPrime * accel);

        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);

        double heading = (omega * config.angularKV)
                + (alpha * config.angularKA)
                + headingKs;

        outResult.pForward = Math.abs(tanPow);
        outResult.pLateral = Math.abs(normPow);
        outResult.pHeading = Math.abs(heading);
        outResult.totalPower = Math.hypot(tanPow, normPow) + Math.abs(heading);
        outResult.maxUtilization = outResult.totalPower;
    }

    private double calculateAngularLimitedTangentialAccel(double currentVel, PathPoint point,
                                                          Path path, double maxAngAccel,
                                                          boolean positiveAccel) {
        double maxPhysicalAccel = config.forwardAccelerationLimit.getIn();
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);
        if (effectiveAngAccelLimit < EPSILON) {
            return 0.0;
        }

        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);
        double alphaBase = fDoublePrime * currentVel * currentVel;

        if (Math.abs(fPrime) < EPSILON) {
            return Math.abs(alphaBase) <= effectiveAngAccelLimit + EPSILON
                    ? maxPhysicalAccel : 0.0;
        }

        double boundA = (-effectiveAngAccelLimit - alphaBase) / fPrime;
        double boundB = (effectiveAngAccelLimit - alphaBase) / fPrime;
        double minAccel = Math.min(boundA, boundB);
        double maxAccel = Math.max(boundA, boundB);

        double angularLimitedAccel = positiveAccel ? maxAccel : -minAccel;
        return Math.min(maxPhysicalAccel, Math.max(0.0, angularLimitedAccel));
    }

    private double signedStatic(double velocity, double accel, double kS) {
        if (Math.abs(velocity) > EPSILON) {
            return Math.signum(velocity) * kS;
        }
        if (Math.abs(accel) > EPSILON) {
            return Math.signum(accel) * kS;
        }
        return 0.0;
    }

    // endregion
}
