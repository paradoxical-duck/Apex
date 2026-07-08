package feedforward;

import androidx.annotation.NonNull;

/**
 * Linear lookup table for generated feedforward states.
 * <p>
 * The generator samples a path into rows of {@link MotionParameters}. The follower later asks
 * for the row at its current path progression; this class linearly interpolates between the two
 * neighboring samples so the target does not jump from row to row.
 */
public class FeedforwardLut {
    /** Ordered samples of the path-relative state [v, a, omega, alpha]. */
    private final MotionParameters[] params;

    /**
     * Wraps generated profile samples.
     *
     * @param generatedParams ordered motion profile samples
     */
    public FeedforwardLut(MotionParameters[] generatedParams) {
        this.params = generatedParams;
    }

    /**
     * Returns an interpolated feedforward target at the requested progression.
     * <p>
     * Interpolation uses {@code y = y0 + t * (y1 - y0)}, where
     * {@code t = (progression - d0) / (d1 - d0)}. Each kinematic component is blended
     * independently.
     *
     * @param progression path progression/distance key to query
     * @return interpolated motion parameters for the follower
     */
    public MotionParameters getFeedforwardParams(double progression) {
        MotionParameters params1 = params[0];
        MotionParameters params2 = params[1];

        // Find the first sample at or beyond the requested progression.
        for (int i = 1; i < params.length; i++) {
            if (progression <= params[i].getProgression()) {
                params1 = params[i - 1];
                params2 = params[i];
                break;
            }
        }

        double d1 = params1.getProgression();
        double d2 = params2.getProgression();

        // Equal progression values can happen at duplicate samples; fall back to the first row.
        double denominator = d2 - d1;
        if (Math.abs(denominator) < 1e-6) {
            return new MotionParameters(
                    params1.getTangentialVel(),
                    params1.getTangentialAccel(),
                    params1.getAngularVel(),
                    params1.getAngularAccel()
            );
        }

        double t = (progression - d1) / denominator;

        return getFeedforwardParams(params1, t, params2);
    }

    /**
     * Blends two neighboring rows of the lookup table.
     *
     * @param params1 lower/previous sample
     * @param t interpolation factor between 0 and 1 in normal use
     * @param params2 upper/next sample
     * @return linearly interpolated parameters
     */
    @NonNull
    private static MotionParameters getFeedforwardParams(MotionParameters params1, double t,
                                                         MotionParameters params2) {
        double interpTransVel =
                params1.getTangentialVel() + t * (params2.getTangentialVel() - params1.getTangentialVel());
        double interpTransAccel =
                params1.getTangentialAccel() + t * (params2.getTangentialAccel() - params1.getTangentialAccel());
        double interpAngVel =
                params1.getAngularVel() + t * (params2.getAngularVel() - params1.getAngularVel());
        double interpAngAccel =
                params1.getAngularAccel() + t * (params2.getAngularAccel() - params1.getAngularAccel());

        return new MotionParameters(
                interpTransVel,
                interpTransAccel,
                interpAngVel,
                interpAngAccel
        );
    }
}
