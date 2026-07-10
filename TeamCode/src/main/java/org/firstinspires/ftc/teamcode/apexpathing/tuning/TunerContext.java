package org.firstinspires.ftc.teamcode.apexpathing.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import controllers.PDSController.PDSCoefficients;
import core.Follower;
import core.FollowerConstants;
import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import util.DistUnit;

public class TunerContext {
    private final LinearOpMode opMode;
    private final FollowerConstants followerConstants;
    private Follower follower;

    public final ElapsedTime timer = new ElapsedTime();

    public double headingP;
    public double headingD;
    public double headingS;
    public double translationP;
    public double translationD;
    public double translationS;
    public double velocityFF;
    public double translationalKA;
    public double angularKV;
    public double angularKA;
    public double velocityFeedbackGain;
    public double angularVelocityFeedbackGain;
    public double maxLateralAccel = 40.0;
    public double headingToleranceDeg;
    public double distanceToleranceIn;

    public TunerContext(LinearOpMode opMode, FollowerConstants followerConstants) {
        this.opMode = opMode;
        this.followerConstants = followerConstants;
    }

    public void loadFrom(FollowerConstants defaults) {
        headingP = defaults.headingCoeffs.kP;
        headingD = defaults.headingCoeffs.kD;
        headingS = defaults.headingCoeffs.kS;
        translationP = defaults.translationalCoeffs.kP;
        translationD = defaults.translationalCoeffs.kD;
        translationS = defaults.translationalCoeffs.kS;
        velocityFF = defaults.translationalKV;
        translationalKA = defaults.translationalKA;
        angularKV = defaults.angularKV;
        angularKA = defaults.angularKA;
        velocityFeedbackGain = defaults.velocityFeedbackGain;
        angularVelocityFeedbackGain = defaults.angularVelocityFeedbackGain;
        headingToleranceDeg = defaults.headingTolerance.getDeg();
        distanceToleranceIn = defaults.distanceTolerance.getIn();
        maxLateralAccel = defaults.strafeAccelerationLimit.getIn() > 10 ?
                defaults.strafeAccelerationLimit.getIn() : 40.0;
    }

    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    public Follower follower() {
        return follower;
    }

    public Telemetry telemetry() {
        return opMode.telemetry;
    }

    public boolean isActive() {
        return opMode.opModeIsActive() && !opMode.isStopRequested();
    }

    public void sleep(long milliseconds) throws InterruptedException {
        opMode.sleep(milliseconds);
    }

    public void resetPose() {
        follower.setPose(new Pose(
                new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)),
                Angle.fromDeg(0)
        ));
    }

    public void driveWithGamepad() {
        follower.teleOpDrive(-opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y,
                -opMode.gamepad1.right_stick_x);
    }

    public void stopDrive() {
        follower.teleOpDrive(0, 0, 0);
    }

    public double manualGuess() {
        return FollowerTuner.kSGuess;
    }

    public void setManualGuess(double value) {
        FollowerTuner.kSGuess = value;
    }

    public void updateFollowerConfig() {
        followerConstants.headingCoeffs = new PDSCoefficients(headingP, headingD, headingS, 0);
        followerConstants.translationalCoeffs = new PDSCoefficients(translationP, translationD,
                translationS, 0);
        followerConstants.translationalKV = velocityFF;
        followerConstants.translationalKA = translationalKA;
        followerConstants.angularKV = angularKV;
        followerConstants.angularKA = angularKA;
        followerConstants.velocityFeedbackGain = velocityFeedbackGain;
        followerConstants.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
        followerConstants.headingTolerance = Angle.fromDeg(headingToleranceDeg);
        followerConstants.distanceTolerance = Dist.fromIn(distanceToleranceIn);
        followerConstants.strafeAccelerationLimit = Dist.fromIn(maxLateralAccel);
    }

    public void saveConstantsToJson() {
        updateFollowerConfig();
        String jsonPayload = "{\n" +
                "  \"drivetrainType\": \"" + followerConstants.drivetrainType.name() + "\",\n" +
                "  \"headingP\": " + headingP + ",\n" +
                "  \"headingD\": " + headingD + ",\n" +
                "  \"headingS\": " + headingS + ",\n" +
                "  \"translationP\": " + translationP + ",\n" +
                "  \"translationD\": " + translationD + ",\n" +
                "  \"translationS\": " + translationS + ",\n" +
                "  \"translationKV\": " + velocityFF + ",\n" +
                "  \"translationKA\": " + translationalKA + ",\n" +
                "  \"angularKV\": " + angularKV + ",\n" +
                "  \"angularKA\": " + angularKA + ",\n" +
                "  \"velocityFeedbackGain\": " + velocityFeedbackGain + ",\n" +
                "  \"AngularVelocityFeedbackGain\": " + angularVelocityFeedbackGain + ",\n" +
                "  \"KC\": " + followerConstants.Kcentripetal + ",\n" +
                "  \"headingToleranceDeg\": " + headingToleranceDeg + ",\n" +
                "  \"distanceToleranceIn\": " + distanceToleranceIn + ",\n" +
                "  \"forwardVelocityLimitInPerSec\": " +
                followerConstants.forwardVelocityLimit.getIn() + ",\n" +
                "  \"forwardAccelerationLimitInPerSec2\": " +
                followerConstants.forwardAccelerationLimit.getIn() + ",\n" +
                "  \"strafeVelocityLimitInPerSec\": " +
                followerConstants.strafeVelocityLimit.getIn() + ",\n" +
                "  \"strafeAccelerationLimitInPerSec2\": " + maxLateralAccel + ",\n" +
                "  \"angularVelocityLimitRadPerSec\": " +
                followerConstants.angularVelocityLimit.getRad() + ",\n" +
                "  \"angularAccelerationLimitRadPerSec2\": " +
                followerConstants.angularAccelerationLimit.getRad() + "\n" +
                "}";

        try {
            File outputFolder = new File("/sdcard/FIRST");
            if (!outputFolder.exists()) outputFolder.mkdirs();
            FileWriter fileWriter = new FileWriter(new File(outputFolder, "FollowerConstants" +
                    ".json"));
            fileWriter.write(jsonPayload);
            fileWriter.close();
        } catch (IOException ignored) {
            telemetry().addLine("WARNING: Values were not saved successfully");
            telemetry().addData("Heading P", headingP);
            telemetry().addData("Heading D", headingD);
            telemetry().addData("Heading S", headingS);
            telemetry().addData("Translation P", translationP);
            telemetry().addData("Translation D", translationD);
            telemetry().addData("Translation S", translationS);
            telemetry().addData("Velocity FF", velocityFF);
            telemetry().addData("Angular kV", angularKV);
            telemetry().addData("Angular kA", angularKA);
            telemetry().addData("Velocity Feedback Gain", velocityFeedbackGain);
            telemetry().addData("Angular Velocity Feedback Gain",
                    angularVelocityFeedbackGain);
            telemetry().addData("Max Lateral Accel", maxLateralAccel);
            telemetry().update();
        }
    }
}
