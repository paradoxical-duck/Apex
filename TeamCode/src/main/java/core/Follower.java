package core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing's main Follower class.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author DrPixelCat
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Xander Haemel 31616 404 Not Found
 */
public class Follower {
    private final FollowerConfig config;
    private final BaseDrivetrain<?> drivetrain;
    private final BaseLocalizer<?> localizer;

    private final double headingTol; // Radians
    private final double distanceTol; // Inches
    private final double velocityLimit; // Inches per second
    private final double velocityS;
    private double lastS = -1.0;
    private long lastNano = -1;

    private final PDSController headingController;
    private final PDSController lateralController;
    private final PDSController driveController;
    private final PDSController velocityController;

    private FollowerMovement currentMovement = null;
    private boolean paused = false;

    // Temporary values to avoid repeated object creation
    PathSegment segment;
    Angle targetHeading;
    Vector targetTurnPoseVec;

    /** Constructs the drivetrain, localizer, and follower from the given {@link ApexConfig}. */
    public Follower(ApexConfig config, HardwareMap hardwareMap) {
        this.drivetrain = config.drivetrainConfig().build(hardwareMap);
        this.localizer = config.localizerConfig().build(hardwareMap);
        this.config = config.followerConfig();

        this.headingTol = this.config.headingTolerance.getRad();
        this.distanceTol = this.config.distanceTolerance.getIn();
        this.velocityLimit = this.config.velocityLimit.getIn();

        this.headingController = new PDSController(this.config.headingCoeffs);
        this.headingController.setAngularController();
        this.lateralController = new PDSController(this.config.lateralCoeffs);
        this.driveController = new PDSController(this.config.driveCoeffs);
        this.velocityController = new PDSController(this.config.velocityCoeffs);

        this.velocityS = driveController.getCoefficients().kS;
    }

    // region General methods
    public void update() {
        if (currentMovement == null || paused) {
            return;
        }

        // Turn power will always be used
        Pose current = getPose();
        Vector currentPos = current.getPos();
        Angle currentHeading = current.getHeading();
        // TODO: Update target heading properly!
        double headingError = targetHeading.getShortestAngleTo(currentHeading).getRad();
        double headingFeedforward = headingController.calculateFromError(headingError);
        long currentNano = System.nanoTime();

        double deltaT_seconds;
        if (lastNano != -1) {
            long elapsedNano = currentNano - lastNano;
            deltaT_seconds = (double) elapsedNano / 1e9;
        } else {
            deltaT_seconds = 0.0;
        }
        lastNano = currentNano;

        if (currentMovement instanceof Turn) {
            if (Math.abs(headingError) < headingTol) {
                this.stop(); return;
            }

            Vector error = targetTurnPoseVec.minus(currentPos);
            double errorMag = error.getMag().getIn();
            if (errorMag > 0) {
                double lateralPower = lateralController.calculateFromError(errorMag);
                Vector feedback = error.normalize().times(lateralPower);
                drivetrain.drive(feedback.getX().getIn(), feedback.getY().getIn(), headingFeedforward);
            } else {
                drivetrain.drive(0, 0, headingFeedforward);
            }
        } else { // TODO: Check heading interpolation here, I have a suspicion that something is messed up
            // Assume it's a Path movement, since that's the only other option.
            // If more movement types are added in the future, this will need to be refactored.
            if (segment == null) {
                this.stop(); return;
            }

            double t = segment.getBestT(currentPos);
            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);
            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);

            // Temporarily flatlined until the LUT provides these targets
            double desiredVelocity = velocityLimit;
            double requiredAccel = 0.0;

            double kappa = segment.getSignedCurvature(t);
            double dKappa = segment.getCurvatureDerivative(t);

            double robotTangentialVel;
            if (deltaT_seconds > 1e-6 && lastS >= 0.0) {
                robotTangentialVel = (lastS - s) / deltaT_seconds;
            } else {
                robotTangentialVel = 0.0;
            }

            lastS = s;

            double omegaTarget = robotTangentialVel * kappa;
            double alphaTarget = (requiredAccel * kappa) + (Math.pow(robotTangentialVel, 2) * dKappa);

            // Heading feedforward
            headingFeedforward += (omegaTarget * config.headingKV) + (alphaTarget * config.headingKA);

            double headingFeedback = headingController.calculateFromError(velVec.getTheta().getRad() - currentHeading.getRad());

            double turnPow = Range.clip(headingFeedback + headingFeedforward, -1.0, 1.0);

            double availableMotorPower = 1.0;
            availableMotorPower -= Math.abs(turnPow); // Heading correction is the highest priority

            // Project field error onto the normal vector to isolate lateral drift
            Vector fieldError = targetPoseVec.minus(currentPos);
            double crossTrackError = fieldError.dot(normal).getIn();
            double lateralFeedbackMag = lateralController.calculateFromError(crossTrackError);

            // Required inward acceleration based on ACTUAL current speed
            double radius = PathSegment.calculateRadiusOfCurvature(velVec, accelVec);
            double requiredLateralAccel = (Math.pow(robotTangentialVel, 2)) / radius;
            double centripetalMag = requiredLateralAccel / config.maxLateralAccel;

            // Combine centripetal feedforward and corrective feedback
            double netLateralMag = centripetalMag + lateralFeedbackMag;
            netLateralMag = Range.clip(netLateralMag, -availableMotorPower, availableMotorPower);

            Vector lateralDriveVec = normal.times(netLateralMag);
            availableMotorPower -= Range.clip(Math.abs(netLateralMag), 0.0,1.0); // Lateral correction is the second priority

            // Project field error onto the tangent vector to isolate forward/backward error
            Vector unitTangent = velVec.normalize();

            double totalTangentPower;
            if (t < 1.0) {
                // We are on the path. Trust the velocity controller and feedforward.
                // BUG FIX: You also used config.lateralKV here instead of config.tangentKV!
                double feedforward = (config.lateralKV * desiredVelocity) + (config.lateralKA * requiredAccel) + driveController.getCoefficients().kS;
                totalTangentPower = velocityController.calculateFromError(desiredVelocity - robotTangentialVel) + feedforward;
            } else {
                // Infinite line fallback for stopping at the exact end coordinate
                Vector endPos = currentMovement.getEndPose().getPos();
                Vector endTangent = segment.getFirstDerivative(1.0).normalize();
                Vector endToRobot = currentPos.minus(endPos);

                double distancePastEnd = endToRobot.dot(endTangent).getIn();
                totalTangentPower = driveController.calculateFromError(-distancePastEnd);
            }

            totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower, availableMotorPower);

            // Apply the power scalar to the purely directional unit tangent
            Vector tangentDriveVec = unitTangent.times(totalTangentPower);
            Vector driveOutput = tangentDriveVec.plus(lateralDriveVec);

            double distanceRemaining = segment.getDistanceToEndIn(targetPoseVec, t);
            if (t >= config.tTolerance && distanceRemaining < distanceTol) {
                this.stop(); return;
            }

            drivetrain.drive(driveOutput.getX().getIn(), driveOutput.getY().getIn(), turnPow);
        }
    }

    /**
     * Checks if the follower is currently performing a movement. This can be used to determine if
     * it's safe to start a new movement or if the current one is still in progress.
     */
    public boolean isBusy() { return currentMovement != null; }

    // region Auto methods
    /**
     * Starts following the given movement.
     *
     * @param movement the {@link FollowerMovement} to be followed
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress! Tip: use follower.isBusy() to check if the follower is currently executing a movement before starting a new one."
            );
        }

        // Update current movement and extract necessary values based on the movement type
        this.currentMovement = movement;
        this.targetHeading = movement.getEndPose().getHeading();
        if (movement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            this.targetTurnPoseVec = turn.getStartPose().getPos();
        } else if (movement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            this.segment = pathSegmentMove.getParametricPath();
        }

        this.headingController.reset();
        this.lateralController.reset();
        this.driveController.reset();
        this.velocityController.reset();
    }

    /** Stops the drivetrain and any ongoing movement. The busy state will be set to false. */
    public void stop() {
        // Reset current movement and temporary values
        this.currentMovement = null;
        this.segment = null;
        this.targetHeading = null;
        this.targetTurnPoseVec = null;

        this.drivetrain.stop();
    }

    /** Stops the drivetrain and any ongoing movement. The busy state will remain true. */
    public void pause() {
        this.paused = true;
        this.drivetrain.stop();
    }

    /** Resumes the current movement if it was paused. If no movement is paused, this method does nothing. */
    public void resume() {
        if (this.paused) {
            this.paused = false;
        }
    }
    // endregion

    // region TeleOp methods
    /**
     * Drives the robot using the provided joystick inputs and robot heading. The joystick inputs
     * are adjusted for field-centric or robot-centric control based on the constants. This method
     * will stop the current movement if one is in progress, as manual control takes priority over
     * following a path.
     *
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
     * @param robotHeading the current heading of the robot in radians, not used for robot centric control
     */
    public void teleOpDrive(double x, double y, double turn, double robotHeading) {
        if (isBusy()) { stop(); }
        drivetrain.drive(x, y, turn, robotHeading);
    }

    /**
     * Drives the robot using the provided joystick inputs. Constants are ignored and robot centric
     * is used because no heading is passed. This method will stop the current movement if one is in
     * progress.
     *
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
     */
    public void teleOpDrive(double x, double y, double turn) { teleOpDrive(x, y, turn, 0); }
    // endregion

    // region Localizer passthrough methods
    /** @return the robot's current pose estimate */
    public Pose getPose() { return localizer.getPose(); }

    /** @param pose the current pose of the robot */
    public void setPose(Pose pose) { localizer.setPose(pose); }

    /**
     * Velocity is expressed in pose form (x and y components in the local robot frame, rotational
     * component in radians per second).
     *
     * @return the robot's current velocity estimate from the localizer
     */
    public Pose getVelocity() { return localizer.getVel(); }

    /**
     * Acceleration is expressed in pose form (x and y components in the local robot frame,
     * rotational component in radians per second squared).
     *
     * @return the robot's current acceleration estimate from the localizer
     */
    public Pose getAcceleration() { return localizer.getAccel(); }
    // endregion
}