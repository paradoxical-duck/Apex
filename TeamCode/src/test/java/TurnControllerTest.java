import org.junit.Test;

import controllers.PDSController;
import controllers.movement.TurnController;
import feedforward.MotionParameters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TurnControllerTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void normalProfileUsesFeedforwardAndVelocityFeedbackOnly() {
        TurnController controller = new TurnController(
                new PDSController.PDSCoefficients(100.0, 100.0, 0.1, 0.0),
                0.1,
                0.01,
                0.2
        );
        MotionParameters targets = new MotionParameters(0.0, 0.0, 2.0, 3.0);

        double output = controller.calculateProfiled(0.5, 1.0, targets, 1.0);

        assertEquals(0.53, output, EPSILON);
        assertFalse(controller.isRecoveringFromOvershoot());
    }

    @Test
    public void staticTermUsesAccelerationAtStartupAndNothingAtRest() {
        TurnController controller = new TurnController(
                new PDSController.PDSCoefficients(5.0, 0.0, 0.1, 0.0),
                0.0,
                0.05,
                0.0
        );

        assertEquals(-0.2, controller.calculateProfiled(
                -0.5, -1.0, new MotionParameters(0.0, 0.0, 0.0, -2.0), 0.0), EPSILON);
        assertEquals(0.0, controller.calculateProfiled(
                -0.5, -1.0, new MotionParameters(), 0.0), EPSILON);
    }

    @Test
    public void overshootLatchesPdsOnlyRecovery() {
        TurnController controller = new TurnController(
                new PDSController.PDSCoefficients(2.0, 0.0, 0.1, 0.0),
                0.9,
                0.9,
                0.9
        );
        MotionParameters aggressiveTargets = new MotionParameters(0.0, 0.0, 1.0, 1.0);

        double correction = controller.calculateProfiled(
                -0.1, 1.0, aggressiveTargets, 0.0);
        assertEquals(-0.3, correction, EPSILON);
        assertTrue(controller.isRecoveringFromOvershoot());

        double correctionAfterCrossingBack = controller.calculateProfiled(
                0.1, 1.0, aggressiveTargets, 0.0);
        assertEquals(0.3, correctionAfterCrossingBack, EPSILON);
        assertTrue(controller.isRecoveringFromOvershoot());
    }

    @Test
    public void quickTurnAndFirstControllerUpdateAreFinite() {
        TurnController quick = new TurnController(
                new PDSController.PDSCoefficients(2.0, 0.0, 0.1, 0.0),
                1.0, 1.0, 1.0);
        assertEquals(0.6, quick.calculateQuick(0.25), EPSILON);

        PDSController derivativeOnly = new PDSController(
                new PDSController.PDSCoefficients(0.0, 1.0, 0.0, 0.0));
        derivativeOnly.reset();
        double firstOutput = derivativeOnly.calculateFromError(1.0);
        assertTrue(Double.isFinite(firstOutput));
        assertEquals(0.0, firstOutput, EPSILON);
    }
}
