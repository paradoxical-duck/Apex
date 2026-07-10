import org.junit.Test;

import controllers.movement.MecanumDriveController;
import controllers.PDSController;
import feedforward.FeedforwardLut;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.Dist;
import geometry.Vector;
import util.DistUnit;

import static org.junit.Assert.assertEquals;

public class DriveFrameAndLutTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void lutClampsAndInterpolatesTraveledDisplacement() {
        MotionParameters start = new MotionParameters(0.0, 0.0, 0.0, 0.0, 0.0);
        MotionParameters end = new MotionParameters(10.0, 4.0, 2.0, -2.0, 10.0);
        FeedforwardLut lut = new FeedforwardLut(new MotionParameters[]{start, end});

        assertEquals(0.0, lut.getFeedforwardParams(-5.0).getTangentialVel(), EPSILON);
        assertEquals(5.0, lut.getFeedforwardParams(5.0).getTangentialVel(), EPSILON);
        assertEquals(1.0, lut.getFeedforwardParams(5.0).getAngularVel(), EPSILON);
        assertEquals(10.0, lut.getFeedforwardParams(15.0).getTangentialVel(), EPSILON);
    }

    @Test
    public void fieldVectorRotatesExactlyIntoRobotFrame() {
        Vector fieldForward = Vector.of(1.0, 0.0, DistUnit.IN);
        Vector robot = MecanumDriveController.fieldToRobotCentric(
                fieldForward, Angle.fromDeg(90.0));

        assertEquals(0.0, robot.getX().getIn(), EPSILON);
        assertEquals(-1.0, robot.getY().getIn(), EPSILON);
    }

    @Test
    public void mecanumScalesStrafeAndPreservesTurnBudget() {
        MecanumDriveController controller = new MecanumDriveController(
                Dist.fromIn(10.0),
                Dist.fromIn(5.0),
                new PDSController.PDSCoefficients(),
                Dist.fromIn(0.25)
        );

        Vector forward = controller.applyMecanumCorrections(
                Vector.of(0.5, 0.0, DistUnit.IN), 0.0);
        assertEquals(0.5, forward.getX().getIn(), EPSILON);
        assertEquals(0.0, forward.getY().getIn(), EPSILON);

        Vector strafe = controller.applyMecanumCorrections(
                Vector.of(0.0, 0.5, DistUnit.IN), 0.0);
        assertEquals(0.0, strafe.getX().getIn(), EPSILON);
        assertEquals(1.0, strafe.getY().getIn(), EPSILON);

        Vector combined = controller.applyMecanumCorrections(
                Vector.of(0.4, 0.4, DistUnit.IN), 0.25);
        assertEquals(0.25, combined.getX().getIn(), EPSILON);
        assertEquals(0.50, combined.getY().getIn(), EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void mecanumRejectsInvalidVelocityLimits() {
        new MecanumDriveController(
                Dist.fromIn(10.0),
                Dist.fromIn(0.0),
                new PDSController.PDSCoefficients(),
                Dist.fromIn(0.25)
        );
    }
}
