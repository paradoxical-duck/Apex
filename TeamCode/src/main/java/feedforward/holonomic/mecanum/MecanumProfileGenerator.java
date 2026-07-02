package feedforward.holonomic.mecanum;

import core.FollowerConstants;
import drivetrains.Mecanum;
import drivetrains.Mecanum.MecanumDirectionalLut.DirectionalKinematics;
import feedforward.BaseProfileGenerator;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

public class MecanumProfileGenerator extends BaseProfileGenerator {
    private final FollowerConstants config;
    private final Mecanum.MecanumDirectionalLut limitCalculator;

    public MecanumProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
        this.limitCalculator = new Mecanum.MecanumDirectionalLut(
                config.forwardVelocityLimit.getIn(),
                config.forwardAccelerationLimit.getIn(),
                config.strafeVelocityLimit.getIn(),
                config.strafeAccelerationLimit.getIn()
        );
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

        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, headingAtPoint);
        double maxPhysicalVel = dirK.maxVel;

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

            if (evaluatePower(mid_v, kappa, fPrime, fDoublePrime, dirK) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    private double evaluatePower(double v, double kappa, double fPrime, double fDoublePrime,
                                 DirectionalKinematics dirK) {
        double boostedKV = config.translationalKV * dirK.velMultiplier;
        double transPower = (v * boostedKV) + config.translationalCoeffs.kS;

        double latPower = Math.abs((v * v * kappa) * config.Kcentripetal);

        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v);
        double headingKs = (Math.abs(omega) > 1e-6) ?
                (Math.signum(omega) * config.headingCoeffs.kS) : 0.0;
        double rotPower =
                Math.abs((omega * config.angularKV) + (alpha * config.angularKA) + headingKs);

        return transPower + latPower + rotPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEnd_in();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        Angle robotHeading = path.getInterpolator().getHeadingTarg(
                s, current.getFirstDerivative(), finalTangent
        );

        DirectionalKinematics dirK = limitCalculator.getKinematics(current.getFirstDerivative(),
                robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a_t);

        double pForward = (v * config.translationalKV * dirK.velMultiplier)
                + (a_t * config.translationalKA * dirK.accelMultiplier)
                + (Math.signum(v) * config.translationalCoeffs.kS);

        double pLateral = (v * v * kappa) * config.Kcentripetal;

        double headingKs = (Math.abs(omega) > 1e-6) ?
                (Math.signum(omega) * config.headingCoeffs.kS) : 0.0;
        double pHeading = (omega * config.angularKV) + (alpha * config.angularKA) + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = Math.abs(pLateral);
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = outResult.pForward + outResult.pLateral + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path,
                                           double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        Vector tangent = point.getFirstDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        Angle robotHeading = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalDecel = limitCalculator.getKinematics(tangent, robotHeading).maxAccel;
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
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle robotHeading = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double boostedKV = config.translationalKV * dirK.velMultiplier;
        double boostedKA = config.translationalKA * dirK.accelMultiplier;

        double vConsumed = (currentVel * boostedKV) + config.translationalCoeffs.kS;
        double latConsumed = Math.abs((currentVel * currentVel * kappa) * config.Kcentripetal);

        double omega = fPrime * currentVel;
        double alphaBase = fDoublePrime * (currentVel * currentVel);
        double headingKs = (Math.abs(omega) > 1e-6) ? config.headingCoeffs.kS : 0.0;
        double rotConsumed =
                Math.abs(omega * config.angularKV) + Math.abs(alphaBase * config.angularKA) + headingKs;

        double vRemaining = Math.max(0.0, 1.0 - (vConsumed + latConsumed + rotConsumed));
        double accelVoltageCost = boostedKA + Math.abs(fPrime * config.angularKA);

        double dynamicAlpha = vRemaining / accelVoltageCost;
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double rotationalTorqueBase =
                    Math.signum(fPrime) * fDoublePrime * (currentVel * currentVel);
            double maxAlpha_at = (effectiveAngAccelLimit - rotationalTorqueBase) / Math.abs(fPrime);

            dynamicAlpha = Math.min(dynamicAlpha, Math.max(0.0, maxAlpha_at));
        }

        return Math.min(dirK.maxAccel, dynamicAlpha);
    }
}