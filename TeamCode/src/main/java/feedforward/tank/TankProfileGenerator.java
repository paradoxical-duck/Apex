package feedforward.tank;

import core.FollowerConstants;
import feedforward.BaseProfileGenerator;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

public class TankProfileGenerator extends BaseProfileGenerator {
    private final FollowerConstants config;

    public TankProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
    }

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point,
                                                    Path path, double maxAngVel,
                                                    double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle headingAtPoint = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalVel = config.forwardVelocityLimit.getIn();

        double effectiveAngVelLimit = Math.min(config.angularVelocityLimit.getRad(), maxAngVel);
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double maxVelFromOmega = effectiveAngVelLimit / Math.abs(fPrime);
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromOmega);
        }

        if (Math.abs(fDoublePrime) > 1e-6) {
            double maxVelFromAlpha = Math.sqrt(effectiveAngAccelLimit / Math.abs(fDoublePrime));
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromAlpha);
        }

        double min_v = 0.0;
        double max_v = maxPhysicalVel;
        int iterations = 5;

        for (int i = 0; i < iterations; i++) {
            double mid_v = (min_v + max_v) / 2.0;

            if (evaluatePower(mid_v, fPrime, fDoublePrime) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    private double evaluatePower(double v, double fPrime, double fDoublePrime) {
        double transPower = Math.abs((v * config.translationalKV) + config.translationalCoeffs.kS);

        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v);
        double headingKs = (Math.abs(omega) > 1e-6) ?
                (Math.signum(omega) * config.headingCoeffs.kS) : 0.0;

        double rotPower =
                Math.abs((omega * config.angularKV) + (alpha * config.angularKA) + headingKs);

        return transPower + rotPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEnd_in();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a_t);

        double pForward =
                (v * config.translationalKV) + (a_t * config.translationalKA) + (Math.signum(v) * config.translationalCoeffs.kS);

        double headingKs = (Math.abs(omega) > 1e-6) ?
                (Math.signum(omega) * config.headingCoeffs.kS) : 0.0;
        double pHeading = (omega * config.angularKV) + (alpha * config.angularKA) + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = 0.0;
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = outResult.pForward + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path,
                                           double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalDecel = config.forwardAccelerationLimit.getIn();
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double rotationalTorqueBase =
                    Math.signum(fPrime) * fDoublePrime * (currentVel * currentVel);
            double maxDecelFromAlpha =
                    (effectiveAngAccelLimit + rotationalTorqueBase) / Math.abs(fPrime);

            maxPhysicalDecel = Math.min(maxPhysicalDecel, Math.max(0.0, maxDecelFromAlpha));
        }

        return maxPhysicalDecel;
    }

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path,
                                              double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double vFwdConsumed = (currentVel * config.translationalKV) + config.translationalCoeffs.kS;

        double omega = fPrime * currentVel;
        double alphaBase = fDoublePrime * (currentVel * currentVel);
        double headingKs = (Math.abs(omega) > 1e-6) ? config.headingCoeffs.kS : 0.0;
        double rotConsumedBase =
                Math.abs(omega * config.angularKV) + Math.abs(alphaBase * config.angularKA) + headingKs;

        double vRemaining = Math.max(0.0, 1.0 - (vFwdConsumed + rotConsumedBase));
        double accelVoltageCost = config.translationalKA + Math.abs(fPrime * config.angularKA);

        double dynamicAlpha = vRemaining / accelVoltageCost;
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double rotationalTorqueBase =
                    Math.signum(fPrime) * fDoublePrime * (currentVel * currentVel);
            double maxAlpha_at = (effectiveAngAccelLimit - rotationalTorqueBase) / Math.abs(fPrime);

            dynamicAlpha = Math.min(dynamicAlpha, Math.max(0.0, maxAlpha_at));
        }

        return Math.min(config.forwardAccelerationLimit.getIn(), dynamicAlpha);
    }
}