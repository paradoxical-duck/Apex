package tuning;

import controllers.PDSController.PDSCoefficients;

public class ForwardCharacterizationPhase extends AxisCharacterizationPhase {
    public ForwardCharacterizationPhase(TunerContext context) {
        super(context, CharacterizationAxis.FORWARD);
    }

    @Override
    protected void applyResult(FeedforwardFit.Result result, double safeVelocity,
                               double safeAcceleration) {
        context.constants.translationalKV = result.kV;
        context.constants.translationalKA = result.kA;
        context.constants.translationalCoeffs = positionGains(result);
        context.constants.velocityFeedbackGain = result.velocityFeedbackGain(0.15);
        context.constants.forwardVelLimitIn = safeVelocity;
        context.constants.forwardAccelLimitIn = safeAcceleration;
        context.constants.Kcentripetal = result.kA;
    }

    @Override
    protected ManualParameter[] createManualParameters() {
        PDSCoefficients pds = context.constants.translationalCoeffs;
        return new ManualParameter[]{
                new ManualParameter("Position kP", () -> pds.kP, value -> pds.kP = value,
                        0.001, 0.0, 2.0),
                new ManualParameter("Position kD", () -> pds.kD, value -> pds.kD = value,
                        0.0005, 0.0, 2.0),
                new ManualParameter("Forward kS", () -> pds.kS, value -> pds.kS = value,
                        0.002, 0.0, 0.5),
                new ManualParameter("Forward kV", () -> context.constants.translationalKV,
                        value -> context.constants.translationalKV = value,
                        0.0005, 0.0, 1.0),
                new ManualParameter("Forward kA", () -> context.constants.translationalKA,
                        value -> context.constants.translationalKA = value,
                        0.0005, 0.0, 1.0),
                new ManualParameter("Velocity feedback", () -> context.constants.velocityFeedbackGain,
                        value -> context.constants.velocityFeedbackGain = value,
                        0.001, 0.0, 2.0),
                new ManualParameter("Forward velocity limit", () -> context.constants.forwardVelLimitIn,
                        value -> context.constants.forwardVelLimitIn = value,
                        1.0, 1.0, 150.0),
                new ManualParameter("Forward acceleration limit", () -> context.constants.forwardAccelLimitIn,
                        value -> context.constants.forwardAccelLimitIn = value,
                        2.0, 1.0, 400.0),
                new ManualParameter("Centripetal kA", () -> context.constants.Kcentripetal,
                        value -> context.constants.Kcentripetal = value,
                        0.0005, 0.0, 1.0)
        };
    }

    @Override protected PDSCoefficients manualPositionCoefficients() {
        return context.constants.translationalCoeffs;
    }
    @Override protected double manualKV() { return context.constants.translationalKV; }
    @Override protected double manualVelocityFeedbackGain() {
        return context.constants.velocityFeedbackGain;
    }
    @Override protected double manualVelocityLimit() {
        return context.constants.forwardVelLimitIn;
    }
}
