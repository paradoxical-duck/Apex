package controllers;

import geometry.Angle;
import geometry.Dist;
import geometry.Vector;

/**
 * A class designed inspired by Wolfpack Machina (18438) to account for mecanum drive inefficiencies
 * and move a robot point-to-point.
 *
 * @author DrPixelCat24
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class DriveController {
    private final double strafePenaltyRatio;
    private final PDSController pds;

    /**
     * @param maxForwardVelIn The maximum forward velocity of the robot in inches per second
     * @param maxStrafeVelIn The maximum strafe velocity of the robot in inches per second
     * @param coefficients The coefficients for the PDkS controller
     */
    public DriveController(double maxForwardVelIn, double maxStrafeVelIn,
                                  PDSController.PDSCoefficients coefficients) {
        this.strafePenaltyRatio = maxForwardVelIn / maxStrafeVelIn;
        this.pds = new PDSController(coefficients);
    }

    public double calculate(double error) {
        return pds.calculateFromError(error);
    }

    public void reset() {
        pds.reset();
    }

    public Vector calculatePointToPointMecanum(Vector targetPos, Vector currentPos, Angle currentHeading) {
        Vector rawFieldVector = calculatePointToPoint(targetPos, currentPos);
        return applyMecanumCorrections(rawFieldVector, currentHeading);
    }

    public Vector calculatePointToPoint(Vector targetPos, Vector currentPos) {
        Vector fieldError = targetPos.minus(currentPos);
        if (fieldError.getMag().getIn() < 0.01) return Vector.zero();

        double basePower = pds.calculateFromError(fieldError.getMag().getIn());
        Vector rawFieldVector = Vector.fromPolar(Dist.fromIn(basePower), fieldError.getTheta());

        return rawFieldVector;
    }

    public Vector applyMecanumCorrections(Vector rawFieldCentricPower, Angle currentHeading) {
        Vector localVector = rawFieldCentricPower.rotate(currentHeading.times(-1.0));

        Vector correctedLocalVector = new Vector(
                Dist.fromIn(localVector.getX().getIn() * strafePenaltyRatio),
                localVector.getY()
        );

        return correctedLocalVector.rotate(currentHeading);
    }
}