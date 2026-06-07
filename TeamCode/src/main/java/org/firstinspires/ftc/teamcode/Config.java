package org.firstinspires.ftc.teamcode;

import controllers.PDSController;
import core.ApexConfig;
import core.FollowerConfig;
import drivetrains.BaseDrivetrainConfig;
import drivetrains.Mecanum;
import localizers.BaseLocalizerConfig;
import localizers.Pinpoint;
import geometry.Angle;
import geometry.Dist;
import util.DistUnit;
import util.MotorFactory;

/**
 * This class extends {@link ApexConfig} and provides the specific constants for the drivetrain,
 * localizer, and follower that we want to use in our OpMode. In this example, we are using a
 * mecanum drivetrain, an OTOS localizer, and a point-to-point follower. You can modify the values in
 * the setDrivetrainConstants(), setLocalizerConstants(), and setFollowerConstants() methods to fit
 * your robot's hardware and tuning preferences.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class Config extends ApexConfig {
    @Override
    public BaseDrivetrainConfig<?> drivetrainConfig() {
        return new Mecanum.Config()
                .setFrontLeftMotor(new MotorFactory("frontLeftMotor"))
                .setBackLeftMotor(new MotorFactory("backLeftMotor"))
                .setFrontRightMotor(new MotorFactory("frontRightMotor").reverse())
                .setBackRightMotor(new MotorFactory("backRightMotor").reverse())
                .setRobotCentric(true)
                .setMaxPower(1.0);
    }

    @Override
    public BaseLocalizerConfig<?> localizerConfig() {
        return new Pinpoint.Config()
                .setName("pinpoint")
                .setOffsets(0, 0, DistUnit.IN)
                .setEncoderDirections(Pinpoint.EncoderDirection.FORWARD, Pinpoint.EncoderDirection.FORWARD)
                .setEncoderResolution(Pinpoint.GoBildaPods.goBILDA_4_BAR_POD);
    }

    @Override
    public FollowerConfig followerConfig() {
        return new FollowerConfig()
                .setHeadingCoeffs(new PDSController.PDSCoefficients())
                .setLateralCoeffs(new PDSController.PDSCoefficients())
                .setDriveCoeffs(new PDSController.PDSCoefficients())
                .setVelocityCoeffs(new PDSController.PDSCoefficients())
                .setFeedforwardCoeffs(0.0, 0.0)
                .setVelocityLimit(Dist.fromIn(20))
                .setHeadingTolerance(Angle.fromDeg(2.0))
                .setDistanceTolerance(Dist.fromIn(1.0))
                .setTTolerance(0.95)
                .setMaxLateralAccel(10.0);
    }
}

/* Tank drivetrain constants
return new Tank.Config()
        .setFrontLeftMotor(new MotorFactory("frontLeftMotor"))
        .setBackLeftMotor(new MotorFactory("backLeftMotor"))
        .setFrontRightMotor(new MotorFactory("frontRightMotor").reverse()) // Don't use back motors for 2 wheel
        .setBackRightMotor(new MotorFactory("backRightMotor").reverse())
        .setRobotCentric(true)
        .setMaxPower(1.0);
 */

/* Swerve drivetrain constants
return new CoaxialSwerve.Config()
        .setFrontLeftMotor(new MotorFactory("frontLeftMotor"))
        .setFrontRightMotor(new MotorFactory("frontRightMotor").reverse())
        .setBackLeftMotor(new MotorFactory("backLeftMotor"))
        .setBackRightMotor(new MotorFactory("backRightMotor").reverse())
        .setServoNames("flServo", "frServo", "blServo", "brServo")
        .setEncoderNames("flEncoder", "frEncoder", "blEncoder", "brEncoder")
        .setMaxPower(1.0)
        .setTrackWidth(Dist.fromMm(288))
        .setWheelbase(Dist.fromMm(288))
        .setRobotCentric(true)
        .setOffsetAngle(Angle.fromDeg(0))
        .setSteeringCoefficients(new PDSController.PDSCoefficients());
*/

/* Kiwi drivetrain constants
return new Kiwi.Config()
        .setFrontLeftMotor(new MotorFactory("frontLeftMotor"))
        .setBackMotor(new MotorFactory("backMotor"))
        .setFrontRightMotor(new MotorFactory("frontRightMotor").reverse())
        .setMaxPower(1.0)
        .setRobotCentric(true);
*/

/* OTOS Constants
return new OTOS.Config() // Tuned for Dylan + Mikey strafer chassis with OTOS, don't change these
        .setName("otos")
        .setOffset(new Pose(Vector.of(227, -16, DistUnit.MM), Angle.fromDeg(0)))
        .setLinearScalar(1.05)
        .setAngularScalar(1.0);
*/