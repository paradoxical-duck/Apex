import org.junit.Test;

import feedforward.FeedforwardLut;
import feedforward.MotionParameters;
import feedforward.angular.TurnProfileGenerator;
import geometry.Angle;
import geometry.Pose;
import geometry.Vector;
import paths.movements.Turn;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TurnProfileGeneratorTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void positiveTurnHasSignedConstrainedProfile() {
        double maxVelocity = 2.0;
        double maxAcceleration = 4.0;
        Turn turn = turn(0.0, 90.0);
        FeedforwardLut profile =
                new TurnProfileGenerator(maxVelocity, maxAcceleration).generate(turn);
        double total = Math.toRadians(90.0);

        MotionParameters start = profile.getFeedforwardParams(0.0);
        MotionParameters end = profile.getFeedforwardParams(total);
        assertEquals(0.0, start.getAngularVel(), EPSILON);
        assertTrue(start.getAngularAccel() > 0.0);
        assertEquals(0.0, end.getAngularVel(), EPSILON);
        assertEquals(0.0, end.getAngularAccel(), EPSILON);

        for (int i = 0; i <= 200; i++) {
            MotionParameters sample = profile.getFeedforwardParams(total * i / 200.0);
            assertTrue(sample.getAngularVel() >= -EPSILON);
            assertTrue(sample.getAngularVel() <= maxVelocity + EPSILON);
            assertTrue(Math.abs(sample.getAngularAccel()) <= maxAcceleration + EPSILON);
        }
    }

    @Test
    public void negativeAndWrappedTurnsPreserveDirection() {
        Turn negative = turn(0.0, -90.0);
        FeedforwardLut negativeProfile = new TurnProfileGenerator(2.0, 4.0).generate(negative);
        MotionParameters negativeStart = negativeProfile.getFeedforwardParams(0.0);
        MotionParameters negativeMiddle =
                negativeProfile.getFeedforwardParams(Math.toRadians(45.0));
        assertTrue(negativeStart.getAngularAccel() < 0.0);
        assertTrue(negativeMiddle.getAngularVel() < 0.0);

        Turn wrapped = turn(170.0, -170.0);
        FeedforwardLut wrappedProfile = new TurnProfileGenerator(2.0, 4.0).generate(wrapped);
        MotionParameters wrappedMiddle =
                wrappedProfile.getFeedforwardParams(Math.toRadians(10.0));
        assertTrue(wrappedMiddle.getAngularVel() > 0.0);
        assertEquals(0.0,
                wrappedProfile.getFeedforwardParams(Math.toRadians(20.0)).getAngularVel(),
                EPSILON);
    }

    @Test
    public void zeroDistanceTurnProducesStationaryProfile() {
        FeedforwardLut profile = new TurnProfileGenerator(2.0, 4.0)
                .generate(turn(45.0, 45.0));
        MotionParameters sample = profile.getFeedforwardParams(100.0);
        assertEquals(0.0, sample.getProgression(), EPSILON);
        assertEquals(0.0, sample.getAngularVel(), EPSILON);
        assertEquals(0.0, sample.getAngularAccel(), EPSILON);
    }

    @Test
    public void profileReservesPowerForBackEmfAccelerationAndStaticFriction() {
        double kV = 0.4;
        double kA = 0.2;
        double kS = 0.1;
        double total = Math.toRadians(170.0);
        FeedforwardLut profile = new TurnProfileGenerator(
                10.0, 10.0, kV, kA, kS).generate(turn(0.0, 170.0));

        MotionParameters start = profile.getFeedforwardParams(0.0);
        assertEquals((1.0 - kS) / kA, start.getAngularAccel(), 1e-5);

        for (int i = 0; i <= 1000; i++) {
            MotionParameters sample = profile.getFeedforwardParams(total * i / 1000.0);
            double velocity = sample.getAngularVel();
            double acceleration = sample.getAngularAccel();
            double motionSign = Math.abs(velocity) > EPSILON
                    ? Math.signum(velocity)
                    : (Math.abs(acceleration) > EPSILON
                    ? Math.signum(acceleration) : 0.0);
            double power = (kV * velocity) + (kA * acceleration) + (kS * motionSign);

            assertTrue(Math.abs(power) <= 1.0 + 1e-5);
            assertTrue(Math.abs(velocity) <= ((1.0 - kS) / kV) + 1e-5);
        }
    }

    private static Turn turn(double startDegrees, double endDegrees) {
        Pose start = new Pose(Vector.zero(), Angle.fromDeg(startDegrees));
        return new Turn(start, Angle.fromDeg(endDegrees));
    }
}
