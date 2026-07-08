package drivetrains;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;
import util.MotorFactory;

/**
 * Coaxial swerve drivetrain controller
 *
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class CoaxialSwerve extends BaseDrivetrain<CoaxialSwerve.Config> {
    private final static double pi2 = 2 * Math.PI;
    private final static double piOver2 = Math.PI / 2;

    private final CRServo flServo, frServo, blServo, brServo;
    private final AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;
    private final PDSController flSteerController, frSteerController, blSteerController,
            brSteerController;
    private final double voltageToRad; // Radians = voltage * this
    private final double offsetAngleRad; // Radians added to encoder angle to get actual wheel angle

    private double lastFlError = 0, lastFrError = 0, lastBlError = 0, lastBrError = 0;

    public CoaxialSwerve(Config config, HardwareMap hardwareMap) {
        super(config, hardwareMap);

        // Make sure all motors, servos, and encoders are configured (front motors are checked in
        // super())
        if (Objects.equals(config.blMotorConfig, null) || Objects.equals(config.brMotorConfig,
                null)) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for a coaxial " +
                            "swerve drivetrain"
            );
        }
        if (Objects.equals(config.flServoName, null) || Objects.equals(config.frServoName, null) ||
                Objects.equals(config.blServoName, null) || Objects.equals(config.brServoName,
                null)) {
            throw new IllegalArgumentException(
                    "Servo names must be provided for all 4 modules in a coaxial swerve drivetrain"
            );
        }
        if (Objects.equals(config.flEncoderName, null) || Objects.equals(config.frEncoderName,
                null) ||
                Objects.equals(config.blEncoderName, null) || Objects.equals(config.brEncoderName
                , null)) {
            throw new IllegalArgumentException(
                    "Encoder names must be provided for all 4 modules in a coaxial swerve " +
                            "drivetrain"
            );
        }

        flServo = hardwareMap.get(CRServo.class, config.flServoName);
        frServo = hardwareMap.get(CRServo.class, config.frServoName);
        blServo = hardwareMap.get(CRServo.class, config.blServoName);
        brServo = hardwareMap.get(CRServo.class, config.brServoName);
        flEncoder = hardwareMap.get(AnalogInput.class, config.flEncoderName);
        frEncoder = hardwareMap.get(AnalogInput.class, config.frEncoderName);
        blEncoder = hardwareMap.get(AnalogInput.class, config.blEncoderName);
        brEncoder = hardwareMap.get(AnalogInput.class, config.brEncoderName);

        flSteerController = new PDSController(config.steeringCoefficients);
        frSteerController = new PDSController(config.steeringCoefficients);
        blSteerController = new PDSController(config.steeringCoefficients);
        brSteerController = new PDSController(config.steeringCoefficients);

        voltageToRad = (2 * Math.PI) / flEncoder.getMaxVoltage();
        offsetAngleRad = config.offsetAngle.getRad();
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Coaxial swerve kinematics explanation:
        // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
        turn *= -1; // Clockwise turn angle

        double strafeFront = y + turn * config.wheelbaseRatio;
        double strafeRear = y - turn * config.wheelbaseRatio;
        double forwardLeft = x + turn * config.trackWidthRatio;
        double forwardRight = x - turn * config.trackWidthRatio;

        double flPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardLeft, 2));
        double frPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardRight, 2));
        double blPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardLeft, 2));
        double brPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardRight, 2));

        double flAngleTarget = Math.atan2(strafeFront, forwardLeft);
        double frAngleTarget = Math.atan2(strafeFront, forwardRight);
        double blAngleTarget = Math.atan2(strafeRear, forwardLeft);
        double brAngleTarget = Math.atan2(strafeRear, forwardRight);

        double flAngle = Angle.normalize(flEncoder.getVoltage() * voltageToRad + offsetAngleRad);
        double frAngle = Angle.normalize(frEncoder.getVoltage() * voltageToRad + offsetAngleRad);
        double blAngle = Angle.normalize(blEncoder.getVoltage() * voltageToRad + offsetAngleRad);
        double brAngle = Angle.normalize(brEncoder.getVoltage() * voltageToRad + offsetAngleRad);

        double flError = wrapError(flAngleTarget, flAngle);
        double frError = wrapError(frAngleTarget, frAngle);
        double blError = wrapError(blAngleTarget, blAngle);
        double brError = wrapError(brAngleTarget, brAngle);

        // If the error > 90 degrees, reverse the wheel direction and subtract 180 from the
        // target angle
        if (Math.abs(flError) > piOver2) {
            flPower *= -1;
            flError -= Math.copySign(Math.PI, flError);
        }
        if (Math.abs(frError) > piOver2) {
            frPower *= -1;
            frError -= Math.copySign(Math.PI, frError);
        }
        if (Math.abs(blError) > piOver2) {
            blPower *= -1;
            blError -= Math.copySign(Math.PI, blError);
        }
        if (Math.abs(brError) > piOver2) {
            brPower *= -1;
            brError -= Math.copySign(Math.PI, brError);
        }

        if (flError != lastFlError) {
            flServo.setPower(flSteerController.calculateFromError(flError));
            lastFlError = flError;
        }
        if (frError != lastFrError) {
            frServo.setPower(frSteerController.calculateFromError(frError));
            lastFrError = frError;
        }
        if (blError != lastBlError) {
            blServo.setPower(blSteerController.calculateFromError(blError));
            lastBlError = blError;
        }
        if (brError != lastBrError) {
            brServo.setPower(brSteerController.calculateFromError(brError));
            lastBrError = brError;
        }

        setPowers(flPower, frPower, blPower, brPower);
    }

    @Override
    public boolean isHolonomic() {
        return true;
    }

    /**
     * @return the error wrapped to the range [-pi, pi] to ensure the shortest path is taken.
     */
    private double wrapError(double target, double current) {
        double errorRaw = target - current;
        return errorRaw - (pi2 * Math.round(errorRaw / pi2));
    }

    /**
     * Configuration class for Mecanum drivetrain.
     */
    public static class Config extends BaseDrivetrainConfig<Config> {
        public String flServoName, frServoName, blServoName, brServoName = "defaultServoName";
        public String flEncoderName, frEncoderName, blEncoderName, brEncoderName =
                "defaultEncoderName";

        public PDSCoefficients steeringCoefficients = new PDSCoefficients();

        public Angle offsetAngle = Angle.zero(); // Encoder value facing forward position
        public Dist wheelbase = Dist.fromIn(14); // Front pod to back pod spacing
        public Dist trackWidth = Dist.fromIn(14); // Left pod to right pod spacing
        public Dist diagonalDist;
        public double wheelbaseRatio; // Wheelbase / diagonal distance
        public double trackWidthRatio; // Track width / diagonal distance

        @Override
        public CoaxialSwerve build(HardwareMap hardwareMap) {
            diagonalDist = wheelbase.hypot(trackWidth);
            wheelbaseRatio = wheelbase.div(diagonalDist).getIn();
            trackWidthRatio = trackWidth.div(diagonalDist).getIn();

            return new CoaxialSwerve(this, hardwareMap);
        }

        /**
         * Sets the front left motor configuration.
         */
        public Config setFrontLeftMotor(MotorFactory motorFactory) {
            this.flMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the front right motor configuration.
         */
        public Config setFrontRightMotor(MotorFactory motorFactory) {
            this.frMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the back left motor configuration.
         */
        public Config setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the back right motor configuration.
         */
        public Config setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the names of the steering servos.
         */
        public Config setServoNames(String fl, String fr, String bl, String br) {
            this.flServoName = fl;
            this.frServoName = fr;
            this.blServoName = bl;
            this.brServoName = br;
            return this;
        }

        /**
         * Sets the names of the steering encoders.
         */
        public Config setEncoderNames(String fl, String fr, String bl, String br) {
            this.flEncoderName = fl;
            this.frEncoderName = fr;
            this.blEncoderName = bl;
            this.brEncoderName = br;
            return this;
        }

        /**
         * Sets the coefficients for the steering PDS controllers.
         */
        public Config setSteeringCoefficients(PDSCoefficients coefficients) {
            this.steeringCoefficients = coefficients;
            return this;
        }

        /**
         * Sets the encoders reported angle when the wheel is facing forward.
         */
        public Config setOffsetAngle(Angle offsetAngle) {
            this.offsetAngle = offsetAngle;
            return this;
        }

        /**
         * Sets the wheelbase of the chassis (front to back pod wheel spacing).
         */
        public Config setWheelbase(Dist wheelbase) {
            this.wheelbase = wheelbase;
            return this;
        }

        /**
         * Sets the track width of the chassis (left to right pod wheel spacing).
         */
        public Config setTrackWidth(Dist trackWidth) {
            this.trackWidth = trackWidth;
            return this;
        }
    }
}