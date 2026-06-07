package paths.heading;

import geometry.Angle;
import geometry.Vector;
import java.util.function.Function;

/**
 * Calculates and manages the target heading of the robot along a path segment.
 * <p>
 * This class is constructed seamlessly by the Builder and evaluates the
 * mathematical target heading dynamically based on the active {@link InterpolationStyle}.
 * <p>
 * Author: DrPixelCat
 */
public class HeadingInterpolator {

    private final InterpolationStyle style;

    // State parameters
    private Angle startHeading;
    private Angle endHeading;
    private Angle customOffset;
    private Function<Double, Angle> customFunction;

    /**
     * Unified constructor for standard interpolation styles.
     * The switch case in getHeading() inherently ignores variables not required for its specific math.
     */
    public HeadingInterpolator(InterpolationStyle style, Angle startHeading, Angle endHeading, Angle customOffset) {
        this.style = style;
        this.startHeading = startHeading != null ? startHeading.copy() : null;
        this.endHeading = endHeading != null ? endHeading.copy() : null;
        this.customOffset = customOffset != null ? customOffset.copy() : null;
    }

    /**
     * Constructor specifically for custom user-defined heading profiles.
     *
     * @throws IllegalStateException if the provided interpolation function is null
     */
    public HeadingInterpolator(Function<Double, Angle> customFunction) {
        this.style = InterpolationStyle.CUSTOM_DIST_FUNCTION;
        this.customFunction = customFunction;
    }

    /**
     * Calculates the target heading for the robot based on the current interpolation style.
     *
     * @param s The distance percentage along the segment [0.0, 1.0]
     * @param pathTangent The 2D forward tangent vector of the path at 's'
     * @return The target Angle the robot should face
     */
    public Angle getHeading(double s, Vector pathTangent) {
        switch (style) {
            case CONSTANT_START_HEADING:
                return startHeading.copy();

            case CONSTANT_END_HEADING:
                return endHeading.copy();

            case TANGENT_FORWARD:
                return pathTangent.getTheta();

            case TANGENT_CUSTOM:
                return pathTangent.getTheta().plus(customOffset);

            case TANGENT_OPTIMAL:
                return calculateOptimalTangent(pathTangent);

            case SMOOTH_START_TO_END:
                return calculateShortestPathLerp(s);

            case CUSTOM_DIST_FUNCTION:
                return customFunction.apply(s);

            default:
                throw new IllegalStateException("Unhandled heading interpolation style: " + style.name());
        }
    }

    /**
     * Aligns with the tangent, but chooses the direction (forward or backward)
     * that minimizes the TOTAL angular travel (entry turn + exit turn).
     */
    private Angle calculateOptimalTangent(Vector tangent) {
        Angle forwardTangent = tangent.getTheta();
        Angle backwardTangent = forwardTangent.plus(Angle.fromRad(Math.PI));

        // Total rotation cost if drive forward
        double entryCostFwd = Math.abs(startHeading.getShortestAngleTo(forwardTangent).getRad());
        double exitCostFwd  = Math.abs(forwardTangent.getShortestAngleTo(endHeading).getRad());
        double totalCostFwd = entryCostFwd + exitCostFwd;

        // Total rotation cost if drive backward
        double entryCostBwd = Math.abs(startHeading.getShortestAngleTo(backwardTangent).getRad());
        double exitCostBwd  = Math.abs(backwardTangent.getShortestAngleTo(endHeading).getRad());
        double totalCostBwd = entryCostBwd + exitCostBwd;

        // Pick the orientation with the smallest total rotational requirement
        return totalCostBwd < totalCostFwd ? backwardTangent : forwardTangent;
    }

    /**
     * Interpolates between startHeading and endHeading via the shortest rotational path,
     * applying a cubic ease-in-out profile so rotational acceleration does not instantly spike.
     */
    private Angle calculateShortestPathLerp(double s) {
        s = Math.max(0.0, Math.min(1.0, s));

        // Apply a Cubic Smoothstep (Ease-In-Out) profile to 's'
        // Equation: f(s) = 3s^2 - 2s^3
        double profiledS = (3.0 * s * s) - (2.0 * s * s * s);

        double diffRad = startHeading.getShortestAngleTo(endHeading).getRad();
        double targetRad = startHeading.getRad() + (diffRad * profiledS);

        return Angle.fromRad(targetRad);
    }
}