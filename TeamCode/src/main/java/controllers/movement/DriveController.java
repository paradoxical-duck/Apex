package controllers.movement;

import controllers.PDSController;
import geometry.Angle;
import geometry.Dist;
import geometry.Vector;

/**
 * Shared translational controller and holonomic power allocator.
 * <p>
 * Field-space path corrections are converted to robot space here. Mecanum allocation additionally
 * applies the robot-relative strafe penalty and measures power using mecanum wheel demand
 * {@code |forward| + |strafe|}. Isotropic allocation is used by swerve and other holonomic drives.
 */
public class DriveController {
    private static final double EPSILON = 1e-9;

    /** One robot-centric command together with the translation budget it consumes. */
    public static final class AllocatedCommand {
        private final Vector robotCommand;
        private final double powerDemand;

        private AllocatedCommand(Vector robotCommand, double powerDemand) {
            this.robotCommand = robotCommand;
            this.powerDemand = powerDemand;
        }

        public Vector getRobotCommand() { return robotCommand; }

        public double getPowerDemand() { return powerDemand; }
    }

    private final double strafePenaltyRatio;
    private final PDSController crossTrackPds;
    private final PDSController endDistancePds;
    private final PDSController turnPositionPds;
    private final Dist tolerance;

    public DriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                           PDSController.PDSCoefficients axialCoefficients,
                           PDSController.PDSCoefficients lateralCoefficients,
                           Dist tolerance, boolean requireMecanumLimits) {
        this.tolerance = tolerance;

        double forwardVelocity = maxForwardVelocity.getIn();
        double strafeVelocity = maxStrafeVelocity.getIn();
        boolean invalidLimits = !Double.isFinite(forwardVelocity) || forwardVelocity <= 0.0 ||
                !Double.isFinite(strafeVelocity) || strafeVelocity <= 0.0;
        if (requireMecanumLimits && invalidLimits) {
            throw new IllegalArgumentException(
                    "Mecanum forward and strafe velocity limits must both be positive."
            );
        }

        strafePenaltyRatio = invalidLimits ? 1.0 : forwardVelocity / strafeVelocity;
        crossTrackPds = new PDSController(lateralCoefficients);
        endDistancePds = new PDSController(axialCoefficients);
        turnPositionPds = new PDSController(axialCoefficients);
    }

    /** Backwards-compatible constructor for callers that intentionally use one gain set. */
    public DriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                           PDSController.PDSCoefficients coefficients, Dist tolerance,
                           boolean requireMecanumLimits) {
        this(maxForwardVelocity, maxStrafeVelocity, coefficients, coefficients, tolerance,
                requireMecanumLimits);
    }

    /** Returns an unallocated field-centric correction toward a fixed position. */
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

    /** Allocates one field-centric stage using mecanum direction-dependent wheel demand. */
    public AllocatedCommand allocateMecanum(Vector fieldCommand, Angle currentHeading,
                                             double availablePower) {
        Vector robotCommand = fieldToRobotCentric(fieldCommand, currentHeading);
        Vector penalized = applyMecanumDirectionalPenalty(robotCommand);
        return limitByDemand(penalized, mecanumWheelDemand(penalized), availablePower);
    }

    /** Allocates one field-centric stage for an isotropic holonomic drivetrain such as swerve. */
    public AllocatedCommand allocateIsotropic(Vector fieldCommand, Angle currentHeading,
                                               double availablePower) {
        Vector robotCommand = fieldToRobotCentric(fieldCommand, currentHeading);
        return limitByDemand(robotCommand, robotCommand.getMag().getIn(), availablePower);
    }

    public static Vector fieldToRobotCentric(Vector fieldVector, Angle currentHeading) {
        return fieldVector.rotate(currentHeading.times(-1.0));
    }

    private Vector applyMecanumDirectionalPenalty(Vector robotCommand) {
        return new Vector(
                robotCommand.getX(),
                Dist.fromIn(robotCommand.getY().getIn() * strafePenaltyRatio)
        );
    }

    private static double mecanumWheelDemand(Vector robotCommand) {
        return Math.abs(robotCommand.getX().getIn()) + Math.abs(robotCommand.getY().getIn());
    }

    private static AllocatedCommand limitByDemand(Vector command, double demand,
                                                   double availablePower) {
        double budget = Math.max(0.0, availablePower);
        if (demand <= EPSILON || budget <= EPSILON) {
            return new AllocatedCommand(Vector.zero(), 0.0);
        }
        if (demand > budget) {
            command = command.times(budget / demand);
            demand = budget;
        }
        return new AllocatedCommand(command, demand);
    }

    public void reset() {
        crossTrackPds.reset();
        endDistancePds.reset();
        turnPositionPds.reset();
    }
}
