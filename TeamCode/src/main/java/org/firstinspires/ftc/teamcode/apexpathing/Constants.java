package org.firstinspires.ftc.teamcode.apexpathing;

import core.ApexConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Mecanum;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;
import util.DistUnit;
import util.MotorFactory;

/**
 * This class extends {@link ApexConstants} and provides the specific drivetrain and localizer
 * configurations for your robot.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Sohum Arora 22985 Paraducks
 */
public class Constants extends ApexConstants {
    @Override
    public BaseDrivetrainConstants<?> drivetrainConstants() {
        return new Mecanum.Constants()
                .setFrontLeftMotor(new MotorFactory("frontLeftMotor"))
                .setBackLeftMotor(new MotorFactory("backLeftMotor"))
                .setFrontRightMotor(new MotorFactory("frontRightMotor").reverse())
                .setBackRightMotor(new MotorFactory("backRightMotor").reverse())
                .setRobotCentric(true)
                .setMaxPower(1.0);
    }

    @Override
    public BaseLocalizerConstants<?> localizerConstants() {
        return new Pinpoint.Constants()
                .setName("pinpoint")
                .setOffsets(0, 0, DistUnit.IN)
                .setEncoderDirections(Pinpoint.EncoderDirection.FORWARD,
                        Pinpoint.EncoderDirection.FORWARD)
                .setEncoderResolution(Pinpoint.GoBildaPods.goBILDA_4_BAR_POD);
    }
}