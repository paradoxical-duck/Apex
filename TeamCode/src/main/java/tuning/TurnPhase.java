package tuning;

import controllers.PDSController.PDSCoefficients;

public class TurnPhase extends DrivePhase {
    public TurnPhase(TunerContext context) {
        super(context, TunerAxis.ANGULAR);
    }

    @Override
    protected void saveResult(FeedforwardCalc.Result result, double safeVelocity, double safeAcceleration) {
        context.constants.angularKV = result.kV;
        context.constants.angularKA = result.kA;
        context.constants.headingCoeffs = makeGains(result);
        context.constants.angularVelocityFeedbackGain = result.velocityGain(0.12);
        context.constants.angularVelLimitRad = safeVelocity;
        context.constants.angularAccelLimitRad = safeAcceleration;
    }

    @Override
    protected TunerValue[] values() {
        PDSCoefficients pds = context.constants.headingCoeffs;
        return new TunerValue[]{
                new TunerValue("Heading kP", () -> pds.kP, value -> pds.kP = value,
                        0.005, 0.0, 5.0),
                new TunerValue("Heading kD", () -> pds.kD, value -> pds.kD = value,
                        0.001, 0.0, 5.0),
                new TunerValue("Heading kS", () -> pds.kS, value -> pds.kS = value,
                        0.002, 0.0, 0.5),
                new TunerValue("Angular kV", () -> context.constants.angularKV,
                        value -> context.constants.angularKV = value,
                        0.002, 0.0, 2.0),
                new TunerValue("Angular kA", () -> context.constants.angularKA,
                        value -> context.constants.angularKA = value,
                        0.001, 0.0, 2.0),
                new TunerValue("Angular velocity feedback",
                        () -> context.constants.angularVelocityFeedbackGain,
                        value -> context.constants.angularVelocityFeedbackGain = value,
                        0.002, 0.0, 5.0),
                new TunerValue("Angular velocity limit",
                        () -> context.constants.angularVelLimitRad,
                        value -> context.constants.angularVelLimitRad = value,
                        0.1, 0.1, 30.0),
                new TunerValue("Angular acceleration limit",
                        () -> context.constants.angularAccelLimitRad,
                        value -> context.constants.angularAccelLimitRad = value,
                        0.2, 0.1, 100.0)
        };
    }

    @Override protected PDSCoefficients manualGains() {
        return context.constants.headingCoeffs;
    }
    @Override protected double manualKV() { return context.constants.angularKV; }
    @Override protected double velocityGain() {
        return context.constants.angularVelocityFeedbackGain;
    }
    @Override protected double speedLimit() {
        return context.constants.angularVelLimitRad;
    }
}
