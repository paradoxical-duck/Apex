import org.junit.Assert;
import org.junit.Test;

import geometry.Angle;
import geometry.Vector;
import paths.builders.HolonomicPathBuilder;
import paths.heading.HolonomicInterpolationStyle;
import paths.heading.HolonomicInterpolator;
import paths.movements.Path;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

public class HolonomicInterpolatorBlendTest {
    private static final double EPSILON_RAD = 1e-6;

    @Test
    public void constantEndHeadingDoesNotTerminalBlendPastEndHeading() {
        HolonomicInterpolator interpolator = new HolonomicInterpolator(
                HolonomicInterpolationStyle.CONSTANT_END_HEADING,
                Angle.fromDeg(0),
                Angle.fromDeg(90),
                null,
                null
        );
        interpolator.setPathLength(100.0);

        Angle target = interpolator.getHeadingTarg(
                0.0,
                Vector.of(1, 0, DistUnit.IN),
                Vector.of(1, 0, DistUnit.IN)
        );

        assertHeadingDeg("CONSTANT_END_HEADING final target", target, 90.0);
    }

    @Test
    public void constantStartHeadingTerminalBlendEndsAtEndHeading() {
        HolonomicInterpolator interpolator = new HolonomicInterpolator(
                HolonomicInterpolationStyle.CONSTANT_START_HEADING,
                Angle.fromDeg(0),
                Angle.fromDeg(90),
                null,
                null
        );
        interpolator.setPathLength(100.0);

        Angle target = interpolator.getHeadingTarg(
                0.0,
                Vector.of(1, 0, DistUnit.IN),
                Vector.of(-1, 0, DistUnit.IN)
        );

        assertHeadingDeg("CONSTANT_START_HEADING final target", target, 90.0);
    }

    @Test
    public void facingPointTerminalBlendEndsAtPathEndHeading() {
        PoseFactory poseFac = new PoseFactory(DistUnit.IN, AngleUnit.DEG);
        Vector pointToFace = poseFac.of(10, 0).getVec();
        Path path = new HolonomicPathBuilder(
                poseFac.of(0, -50, 90),
                poseFac.of(0, 50, 0))
                .interpolateWith(HolonomicInterpolationStyle.FACING_POINT, pointToFace)
                .quickBuild();

        double pathLength = path.getParametricPath().getLengthIn();
        Vector startTangent = path.getParametricPath().getFirstDerivative(0.0);
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle startTarget = path.getInterpolator().getHeadingTarg(
                pathLength,
                startTangent,
                finalTangent
        );
        Angle endTarget = path.getInterpolator().getHeadingTarg(
                0.0,
                finalTangent,
                finalTangent
        );

        Angle expectedStartFacing =
                pointToFace.minus(path.getParametricPath().getPosition(0.0)).getTheta();
        assertHeading("FACING_POINT start target", startTarget, expectedStartFacing);
        assertHeadingDeg("FACING_POINT path end pose", path.getEndPose().getHeading(), 0.0);
        assertHeadingDeg("FACING_POINT final target", endTarget, 0.0);
    }

    private void assertHeadingDeg(String label, Angle actual, double expectedDeg) {
        assertHeading(label, actual, Angle.fromDeg(expectedDeg));
    }

    private void assertHeading(String label, Angle actual, Angle expected) {
        double error = actual.getShortestAngleTo(expected).getRad();
        Assert.assertEquals(label, 0.0, error, EPSILON_RAD);
    }
}
