package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import geometry.Angle;
import geometry.Vector;
import util.MotorFactory;

/**
 * Mecanum drivetrain controller
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Mecanum extends BaseDrivetrain<Mecanum.Constants> {
    public Mecanum(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.MECANUM);

        if (Objects.equals(constants.blMotorConfig, null) ||
                Objects.equals(constants.brMotorConfig, null)) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for a mecanum drivetrain"
            );
        }
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Mecanum kinematics explanation: https://www.youtube.com/watch?v=gnSW2QpkGXQ
        setPowers(x - y - turn, x + y + turn,
                x + y - turn, x - y + turn
        );
    }

    /** Configuration class for Mecanum drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        @Override
        public Mecanum build(HardwareMap hardwareMap) { return new Mecanum(this, hardwareMap); }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(MotorFactory motorFactory) {
            this.flMotorConfig = motorFactory;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(MotorFactory motorFactory) {
            this.frMotorConfig = motorFactory;
            return this;
        }

        /** Sets the back left motor configuration. */
        public Constants setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        /** Sets the back right motor configuration. */
        public Constants setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }
    }

    public static class DirectionalLut {
        public static class DirectionalKinematics {
            public final double maxVel;
            public final double maxAccel;
            public final double velMultiplier;
            public final double accelMultiplier;

            public DirectionalKinematics(double maxVel, double maxAccel, double velMultiplier,
                                         double accelMultiplier) {
                this.maxVel = maxVel;
                this.maxAccel = maxAccel;
                this.velMultiplier = velMultiplier;
                this.accelMultiplier = accelMultiplier;
            }
        }

        private final DirectionalKinematics[] lut = new DirectionalKinematics[360];

        /**
         * Precomputes the entire 360-degree kinematic capability of the mecanum drive.
         *
         * @param maxFwdVel Absolute maximum forward velocity (in/s)
         * @param maxFwdAccel Absolute maximum forward acceleration (in/s^2)
         * @param maxSfeVel Absolute maximum strafing velocity (in/s)
         * @param maxSfeAccel Absolute maximum strafing acceleration (in/s^2)
         */
        public DirectionalLut(double maxFwdVel, double maxFwdAccel, double maxSfeVel,
                                     double maxSfeAccel) {
            for (int i = 0; i < 360; i++) {
                double theta = Math.toRadians(i);

                double absX = Math.abs(Math.cos(theta));
                double absY = Math.abs(Math.sin(theta));

                double maxVel = 1.0 / ((absY / maxFwdVel) + (absX / maxSfeVel));
                double maxAccel = 1.0 / ((absY / maxFwdAccel) + (absX / maxSfeAccel));

                double velMultiplier = maxFwdVel / maxVel;
                double accelMultiplier = maxFwdAccel / maxAccel;

                lut[i] = new DirectionalKinematics(maxVel, maxAccel, velMultiplier,
                        accelMultiplier);
            }
        }

        public DirectionalKinematics getKinematics(Vector globalDriveVector, Angle currentHeading) {
            if (globalDriveVector.getMagSq().getIn() < 1e-9) {
                return lut[0]; // Default to forward limits
            }

            Vector localVector = globalDriveVector.rotate(currentHeading.times(-1.0));
            double rads = localVector.getTheta().getRad();

            int degrees = (int) Math.round(Math.toDegrees(rads));
            degrees = ((degrees % 360) + 360) % 360;

            return lut[degrees];
        }
    }
}