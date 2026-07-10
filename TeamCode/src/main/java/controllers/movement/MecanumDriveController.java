package controllers.movement;

import controllers.PDSController;
import geometry.Angle;
import geometry.Dist;
import geometry.Vector;

/**
 * A class designed inspired by Wolfpack Machina (18438) to account for mecanum drive inefficiencies
 * and move a robot point-to-point
 *
 * @author DrPixelCat24
 */
public class MecanumDriveController {
    private final double strafePenaltyRatio;
    private final PDSController crossTrackPds;
    private final PDSController endDistancePds;
    private final PDSController turnPositionPds;
    public final Dist tolerance;

    /**
     * @param maxForwardVelocity The maximum forward velocity of the robot
     * @param maxStrafeVelocity  The maximum strafe velocity of the robot
     * @param PDSCoefficients    The coefficients for the PDkS controller
     * @param tolerance          The distance at which power is no longer applied
     */
    public MecanumDriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                                  PDSController.PDSCoefficients PDSCoefficients, Dist tolerance) {
        this(maxForwardVelocity, maxStrafeVelocity, PDSCoefficients, tolerance, true);
    }

    public MecanumDriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                                  PDSController.PDSCoefficients PDSCoefficients, Dist tolerance,
                                  boolean requireMecanumLimits) {
        this.tolerance = tolerance;
        double forwardVelocity = maxForwardVelocity.getIn();
        double strafeVelocity = maxStrafeVelocity.getIn();
        boolean invalidLimits = !Double.isFinite(forwardVelocity) || forwardVelocity <= 0.0 ||
                !Double.isFinite(strafeVelocity) || strafeVelocity <= 0.0;
        if (requireMecanumLimits && invalidLimits) {
            throw new IllegalArgumentException(
                    "Forward and strafe velocity limits must both be positive."
            );
        }

        this.strafePenaltyRatio = invalidLimits ? 1.0 : forwardVelocity / strafeVelocity;
        this.crossTrackPds = new PDSController(PDSCoefficients);
        this.endDistancePds = new PDSController(PDSCoefficients);
        this.turnPositionPds = new PDSController(PDSCoefficients);
    }

    /** Returns a field-centric correction toward a fixed position. */
    public Vector calculatePointToPoint(Vector targetPos, Vector currentPos) {
        Vector fieldError = targetPos.minus(currentPos);
        if (fieldError.getMag().getIn() < tolerance.getIn()) return Vector.zero();

        double basePower = turnPositionPds.calculateFromError(fieldError.getMag().getIn());
        return Vector.fromPolar(Dist.fromIn(basePower), fieldError.getTheta());
    }

    public double calculateCrossTrack(double error) {
        return crossTrackPds.calculateFromError(error);
    }

    public double calculateEndDistance(double error) {
        return endDistancePds.calculateFromError(error);
    }

    /** Converts a field-centric vector into the robot's local coordinate frame. */
    public static Vector fieldToRobotCentric(Vector fieldVector, Angle currentHeading) {
        return fieldVector.rotate(currentHeading.times(-1.0));
    }

    /**
     * Applies robot-relative mecanum strafe compensation while preserving the turn power budget.
     */
    public Vector applyMecanumCorrections(Vector robotCentricPower, double turnPower) {
        Vector corrected = new Vector(
                robotCentricPower.getX(),
                Dist.fromIn(robotCentricPower.getY().getIn() * strafePenaltyRatio)
        );

        double availableTranslationPower = Math.max(0.0, 1.0 - Math.abs(turnPower));
        double wheelDemand = Math.abs(corrected.getX().getIn())
                + Math.abs(corrected.getY().getIn());
        if (wheelDemand > availableTranslationPower && wheelDemand > 1e-9) {
            corrected = corrected.times(availableTranslationPower / wheelDemand);
        }

        return corrected;
    }

    public void reset() {
        crossTrackPds.reset();
        endDistancePds.reset();
        turnPositionPds.reset();
    }
}
