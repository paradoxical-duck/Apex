package tuning;

import controllers.PDSController.PDSCoefficients;

public class AngularCharacterizationPhase extends AxisCharacterizationPhase {
    public AngularCharacterizationPhase(TunerContext context) {
        super(context, CharacterizationAxis.ANGULAR);
    }

    @Override
    protected void applyResult(FeedforwardFit.Result result, double safeVelocity,
                               double safeAcceleration) {
        context.constants.angularKV = result.kV;
        context.constants.angularKA = result.kA;
        context.constants.headingCoeffs = positionGains(result);
        context.constants.angularVelocityFeedbackGain = result.velocityFeedbackGain(0.12);
        context.constants.angularVelLimitRad = safeVelocity;
        context.constants.angularAccelLimitRad = safeAcceleration;
    }

    @Override
    protected ManualParameter[] createManualParameters() {
        PDSCoefficients pds = context.constants.headingCoeffs;
        return new ManualParameter[]{
                new ManualParameter("Heading kP", () -> pds.kP, value -> pds.kP = value,
                        0.005, 0.0, 5.0),
                new ManualParameter("Heading kD", () -> pds.kD, value -> pds.kD = value,
                        0.001, 0.0, 5.0),
                new ManualParameter("Heading kS", () -> pds.kS, value -> pds.kS = value,
                        0.002, 0.0, 0.5),
                new ManualParameter("Angular kV", () -> context.constants.angularKV,
                        value -> context.constants.angularKV = value,
                        0.002, 0.0, 2.0),
                new ManualParameter("Angular kA", () -> context.constants.angularKA,
                        value -> context.constants.angularKA = value,
                        0.001, 0.0, 2.0),
                new ManualParameter("Angular velocity feedback",
                        () -> context.constants.angularVelocityFeedbackGain,
                        value -> context.constants.angularVelocityFeedbackGain = value,
                        0.002, 0.0, 5.0),
                new ManualParameter("Angular velocity limit",
                        () -> context.constants.angularVelLimitRad,
                        value -> context.constants.angularVelLimitRad = value,
                        0.1, 0.1, 30.0),
                new ManualParameter("Angular acceleration limit",
                        () -> context.constants.angularAccelLimitRad,
                        value -> context.constants.angularAccelLimitRad = value,
                        0.2, 0.1, 100.0)
        };
    }

    @Override protected PDSCoefficients manualPositionCoefficients() {
        return context.constants.headingCoeffs;
    }
    @Override protected double manualKV() { return context.constants.angularKV; }
    @Override protected double manualVelocityFeedbackGain() {
        return context.constants.angularVelocityFeedbackGain;
    }
    @Override protected double manualVelocityLimit() {
        return context.constants.angularVelLimitRad;
    }
}
