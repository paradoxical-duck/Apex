package core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import controllers.movement.MecanumDriveController;
import controllers.PDSController;
import controllers.movement.TurnController;
import drivetrains.BaseDrivetrain;
import drivetrains.CoaxialSwerve;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.Dist;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.callbacks.Callback;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing main Follower class.
 * Handles the execution of generated paths and turns using kinematic feedforward and feedback
 * controllers.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author DrPixelCat
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Xander Haemel 31616 404 Not Found
 */
public class Follower {
    private final FollowerConstants config;
    private final BaseDrivetrain<?> drivetrain;
    private final BaseLocalizer<?> localizer;

    private final double headingTol;
    private final double distanceTol;

    private double lastS = -1.0;
    private long lastNano = -1;
    private Angle lastHeading = null; // Tracks heading between ticks for angular callback sweeps

    private final PDSController headingController;
    private final TurnController turnController;
    private final MecanumDriveController mecanumDriveController;
    private final double velocityFeedbackGain;

    private FollowerMovement currentMovement = null;
    private boolean paused = false;

    PathSegment segment;
    Angle targetHeading;
    Vector targetTurnPoseVec;
    double turnDirection;
    double turnTotalDisplacement;

    /**
     * Constructs the drivetrain, localizer, and follower controllers from the configuration.
     *
     * @param config      The Apex configuration file containing hardware and tuning settings.
     * @param hardwareMap The active OpMode hardware map.
     */
    public Follower(ApexConfig config, HardwareMap hardwareMap) {
        drivetrain = config.drivetrainConfig().build(hardwareMap);
        localizer = config.localizerConfig().build(hardwareMap);
        this.config = config.followerConfig();

        headingTol = this.config.headingTolerance.getRad();
        distanceTol = this.config.distanceTolerance.getIn();

        headingController = new PDSController(this.config.headingCoeffs);
        headingController.setAngularController();
        turnController = new TurnController(
                this.config.headingCoeffs,
                this.config.angularKV,
                this.config.angularKA,
                this.config.angularVelocityFeedbackGain
        );

        mecanumDriveController = new MecanumDriveController(
                this.config.forwardVelocityLimit,
                this.config.strafeVelocityLimit,
                this.config.translationalCoeffs,
                Dist.fromIn(0.25),
                drivetrain instanceof Mecanum || drivetrain instanceof DualActuated
        );
        velocityFeedbackGain = this.config.velocityFeedbackGain;
    }

    // region Callbacks

    /**
     * Evaluates all callbacks attached to the current movement and executes them if their
     * conditions are met.
     * * @param t The current geometric progression percentage [0.0, 1.0]. Pass -1.0 for turns.
     *
     * @param currentHeading The robot's current field orientation.
     */
    private void processCallbacks(double s, Angle currentHeading) {
        Callback[] callbacks = null;
        if (currentMovement instanceof Path) {
            callbacks = ((Path) currentMovement).getCallbacks();
        } else if (currentMovement instanceof Turn) {
            callbacks = ((Turn) currentMovement).getCallbacks();
        }

        if (callbacks != null) {
            for (Callback cb : callbacks) {
                if (cb.isTriggered()) continue;

                boolean shouldTrigger = false;

                if (cb.getType() == Callback.CallbackType.DISTANCE) {
                    if (s >= cb.getS() && s >= 0.0) {
                        shouldTrigger = true;
                    }
                } else if (cb.getType() == Callback.CallbackType.ANGLE) {
                    double error =
                            Math.abs(currentHeading.getShortestAngleTo(cb.getTheta()).getRad());

                    // Trigger if resting within 1 degree of target
                    if (error < Math.toRadians(1.0)) {
                        shouldTrigger = true;
                    }
                    // Trigger if the target angle was swept past between the last tick and this
                    // tick
                    else if (lastHeading != null) {
                        double tickSweep = lastHeading.getShortestAngleTo(currentHeading).getRad();
                        double targetSweep = lastHeading.getShortestAngleTo(cb.getTheta()).getRad();

                        // If sweeps are in the same direction AND the tick sweep is larger, it
                        // was crossed
                        if (Math.signum(tickSweep) == Math.signum(targetSweep) && Math.abs(targetSweep) <= Math.abs(tickSweep)) {
                            shouldTrigger = true;
                        }
                    }
                }

                if (shouldTrigger) {
                    cb.getAction().run();
                    cb.setTriggered(true);
                }
            }
        }
        lastHeading = currentHeading;
    }

    /**
     * The main execution loop of the follower.
     * Must be called continuously during the active OpMode loop to drive the robot along the path.
     */
    public void update() {
        // Exit early if nothing is running
        if (currentMovement == null || paused) {
            return;
        }

        Pose current = getPose();
        Vector currentPos = current.getVec();
        Angle currentHeading = current.getHeading();

        long currentNano = System.nanoTime();

        // Calculate delta time for velocity feedback
        double deltaT_seconds = (lastNano != -1) ? (currentNano - lastNano) / 1e9 : 0.0;
        lastNano = currentNano;

        // region Turn Execution
        if (currentMovement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            double headingError = currentHeading.getShortestAngleTo(targetHeading).getRad();

            // Process angular callbacks
            processCallbacks(-1.0, currentHeading);

            // Require both positional accuracy and low angular velocity to prevent momentum
            // overshoot
            double currentAngularVel = localizer.getVel().getHeading().getRad();
            if (Math.abs(headingError) < headingTol && Math.abs(currentAngularVel) < 0.05) {
                this.stop();
                return;
            }

            double totalTurnPower;
            if (turn.getFeedforwardLut() == null || turnTotalDisplacement < 1e-9) {
                totalTurnPower = turnController.calculateQuick(headingError);
            } else {
                double signedTravel = turn.getStartPose().getHeading()
                        .getShortestAngleTo(currentHeading).getRad() * turnDirection;
                double angularDisplacement = Range.clip(
                        signedTravel, 0.0, turnTotalDisplacement);
                MotionParameters turnTargets = turn.getFeedforwardLut()
                        .getFeedforwardParams(angularDisplacement);
                totalTurnPower = turnController.calculateProfiled(
                        headingError,
                        turnDirection,
                        turnTargets,
                        currentAngularVel
                );
            }

            Vector error = targetTurnPoseVec.minus(currentPos);
            double errorMag = error.getMag().getIn();

            // Hold xy position actively while turning
            if (drivetrain.isHolonomic() && errorMag > distanceTol) {
                Vector fieldFeedback = mecanumDriveController.calculatePointToPoint(
                        targetTurnPoseVec, currentPos);
                Vector robotFeedback = prepareHolonomicCommand(
                        fieldFeedback, currentHeading, totalTurnPower);
                drivetrain.drive(robotFeedback.getX().getIn(),
                        robotFeedback.getY().getIn(), totalTurnPower);
            } else {
                drivetrain.drive(0, 0, totalTurnPower);
            }

        } else if (segment == null) {
            this.stop();
            // region Holonomic Following
        } else if (drivetrain.isHolonomic()) {
            // Retrieve path geometry at closest point
            double t = segment.getBestT(currentPos);

            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);
            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);
            Vector unitTangent = velVec.normalize();
            Vector endTangent = segment.getFirstDerivative(1.0).normalize();

            // Process scheduled distance and angular callbacks
            processCallbacks(s / segment.getLengthIn(), currentHeading);

            Vector robotVel = localizer.getVel().getVec();
            double distanceRemaining = segment.getDistanceToEndIn(targetPoseVec, t);
            double kappa = segment.getSignedCurvature(t);
            double dKappa = segment.getCurvatureDerivative(t);

            Path path = (Path) currentMovement;
            boolean isProfiled = path.isProfiled();
            double distanceTraveled = path.getParametricPath().getLengthIn() - s;
            MotionParameters targets = isProfiled ?
                    path.getFeedforwardLut().getFeedforwardParams(distanceTraveled) : null;

            boolean isSwerve = drivetrain instanceof CoaxialSwerve;

            double robotTangentialVel = (deltaT_seconds > 1e-6 && lastS >= 0.0) ?
                    (lastS - s) / deltaT_seconds : 0.0;
            lastS = s;

            // Calculate heading power allocation
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec, endTangent);
            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, endTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                    endTangent);

            double headingFF = 0.0;
            if (isProfiled) {
                double omegaTarget = fPrime * robotTangentialVel;
                double alphaTarget =
                        (fDoublePrime * (robotTangentialVel * robotTangentialVel)) + (fPrime * targets.getTangentialAccel());

                headingFF = (omegaTarget * config.angularKV) + (alphaTarget * config.angularKA);
                if (Math.abs(omegaTarget) > 1e-6) {
                    headingFF += Math.signum(omegaTarget) * config.headingCoeffs.kS;
                }
            }

            double headingFeedback =
                    headingController.calculateFromError(headingTarg.getRad() - currentHeading.getRad());
            double turnPow = Range.clip(headingFeedback + headingFF, -1.0, 1.0);
            double availableMotorPower = 1.0 - Math.abs(turnPow);

            // Calculate lateral cross track power allocation
            Vector positionalError = targetPoseVec.minus(currentPos);
            double crossTrackError = positionalError.dot(normal).getIn();
            double lateralFeedbackMag =
                    mecanumDriveController.calculateCrossTrack(crossTrackError);

            double requiredLateralAccel = (robotTangentialVel * robotTangentialVel) * kappa;
            double centripetalMag = requiredLateralAccel * config.Kcentripetal;

            double netLateralMag = Range.clip(centripetalMag + lateralFeedbackMag,
                    -availableMotorPower, availableMotorPower);
            Vector rawLateralDriveVec = normal.times(netLateralMag);

            // Calculate tangential forward power allocation
            double tangentBudget;
            if (isSwerve) {
                tangentBudget =
                        Math.sqrt((availableMotorPower * availableMotorPower) - (netLateralMag * netLateralMag));
            } else {
                tangentBudget = availableMotorPower - Math.abs(netLateralMag);
            }

            double totalTangentPower;
            if (t < 1.0) {
                if (isProfiled) {
                    double feedforward = (config.translationalKV * targets.getTangentialVel()) +
                            (config.translationalKA * targets.getTangentialAccel()) +
                            (Math.signum(targets.getTangentialVel()) * config.translationalCoeffs.kS);

                    totalTangentPower =
                            ((targets.getTangentialVel() - robotTangentialVel) * velocityFeedbackGain) + feedforward; //TODO: Verify p only feedback performance, compare to SquID
                    if (path.isAccelBoosted()) {
                        totalTangentPower = Math.min(
                                totalTangentPower,
                                mecanumDriveController.calculateEndDistance(distanceRemaining));
                    }
                } else {
                    double decelPower =
                            mecanumDriveController.calculateEndDistance(distanceRemaining);
                    double percentage = 1.0 - (s / path.getParametricPath().getLengthIn());
                    double percentageClipped = Math.min(Math.max(percentage, 0.0), 1.0);
                    double maxVel = path.getQuickVelocityLimit(percentageClipped,
                            config.forwardVelocityLimit.getIn());
                    double velError = maxVel - robotTangentialVel;
                    double accelPower = (maxVel * config.translationalKV)
                            + (Math.signum(maxVel) * config.translationalCoeffs.kS)
                            + (velError * velocityFeedbackGain);
                    totalTangentPower = Math.min(accelPower, decelPower);
                }
            } else {
                // Apply reverse feedback if robot drifts past the final point
                double distancePastEnd = currentPos.minus(targetPoseVec).dot(endTangent).getIn();
                totalTangentPower =
                        mecanumDriveController.calculateEndDistance(-distancePastEnd);
            }

            totalTangentPower = Range.clip(totalTangentPower, -tangentBudget, tangentBudget);
            Vector rawTangentDriveVec = unitTangent.times(totalTangentPower);

            // Combine and filter output
            Vector rawTranslationalOutput = rawTangentDriveVec.plus(rawLateralDriveVec);
            Vector finalDriveOutput = prepareHolonomicCommand(
                    rawTranslationalOutput, currentHeading, turnPow);

            // Check stop condition and drive hardware
            if (distanceRemaining < distanceTol && robotVel.getMagSq().getIn() < 25) {
                stop();
                return;
            }

            drivetrain.drive(finalDriveOutput.getX().getIn(), finalDriveOutput.getY().getIn(),
                    turnPow);
            // region Tank Following
        } else {
            // Process tank driving via Ramsete controller
            double t = segment.getBestT(currentPos);

            // Process scheduled distance and angular callbacks
            processCallbacks(t, currentHeading);

            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector robotVel = localizer.getVel().getVec();

            Path path = (Path) currentMovement;
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec,
                    segment.getFirstDerivative(1.0));
            double distanceTraveled = path.getParametricPath().getLengthIn() - s;
            MotionParameters targets =
                    path.getFeedforwardLut().getFeedforwardParams(distanceTraveled);

            double v_d = targets.getTangentialVel();
            double a_d = targets.getTangentialAccel();
            double omega_d = targets.getAngularVel();
            double alpha_d = targets.getAngularAccel();

            // Transform global error to robot local frame
            Vector globalError = targetPoseVec.minus(currentPos);
            Vector localError = globalError.rotate(Angle.fromRad(-currentHeading.getRad()));

            double e_x = localError.getX().getIn();
            double e_y = localError.getY().getIn();
            double e_theta = currentHeading.getShortestAngleTo(headingTarg).getRad();

            // Calculate non linear Ramsete gains
            double b = 2.0;
            double zeta = 0.7;
            double k = 2.0 * zeta * Math.sqrt(Math.pow(omega_d, 2) + b * Math.pow(v_d, 2));
            double sinc = (Math.abs(e_theta) < 1e-6) ? 1.0 : (Math.sin(e_theta) / e_theta);

            double v_cmd = v_d * Math.cos(e_theta) + k * e_x;
            double w_cmd = omega_d + k * e_theta + b * v_d * sinc * e_y;

            // Convert velocity commands to motor power using feedforward constants
            double totalTangentPower =
                    (v_cmd * config.translationalKV) + (a_d * config.translationalKA) + (Math.signum(v_cmd) * config.translationalCoeffs.kS);
            double turnPow = (w_cmd * config.angularKV) + (alpha_d * config.angularKA);
            turnPow += Math.signum(turnPow) * config.headingCoeffs.kS;

            double availableMotorPower = 1.0;
            turnPow = Range.clip(turnPow, -availableMotorPower, availableMotorPower);
            availableMotorPower -= Math.abs(turnPow);
            totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower,
                    availableMotorPower);

            if (s < distanceTol && robotVel.getMagSq().getIn() < 16) {
                stop();
                return;
            }

            drivetrain.drive(totalTangentPower, 0.0, turnPow);
        }
    }

    private boolean isMecanumDrive() {
        return drivetrain instanceof Mecanum ||
                (drivetrain instanceof DualActuated && drivetrain.isHolonomic());
    }

    /** Converts one field-centric translation into the robot frame at the actuation boundary. */
    private Vector prepareHolonomicCommand(Vector fieldCommand, Angle currentHeading,
                                            double turnPower) {
        Vector robotCommand = MecanumDriveController.fieldToRobotCentric(
                fieldCommand, currentHeading);
        if (isMecanumDrive()) {
            robotCommand = mecanumDriveController.applyMecanumCorrections(
                    robotCommand, turnPower);
        }
        return robotCommand;
    }

    //region Initialize Sequence

    /**
     * Starts following the given movement.
     *
     * @param movement The movement object to be executed.
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress " +
                            "Tip use follower.isBusy() to check if the follower is currently " +
                            "executing a movement before starting a new one."
            );
        }

        this.currentMovement = movement;
        this.currentMovement.setStarted(true);
        this.currentMovement.setEnded(false);
        this.targetHeading = movement.getEndPose().getHeading();

        if (movement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            this.targetTurnPoseVec = turn.getStartPose().getVec();
            double signedTurn = turn.getStartPose().getHeading()
                    .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
            this.turnDirection = Math.signum(signedTurn);
            this.turnTotalDisplacement = Math.abs(signedTurn);
        } else if (movement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            this.segment = pathSegmentMove.getParametricPath();
            if (drivetrain instanceof DualActuated) {
                if (pathSegmentMove.getPathType() == Path.PathType.HOLONOMIC) {
                    ((DualActuated) drivetrain).activateHolonomicState();
                } else {
                    ((DualActuated) drivetrain).activateTractionState();
                }
            }
        }

        headingController.reset();
        turnController.reset();
        mecanumDriveController.reset();
        lastS = -1.0;
        lastNano = -1;
        paused = false;

        // Reset sweeping tracker for angular callbacks so it doesn't instantly trigger on path
        // start
        lastHeading = null;
    }

    // region Public Methods

    /**
     * Instantly stops the drivetrain and ends any ongoing movement.
     */
    public void stop() {
        if (this.currentMovement != null) {
            this.currentMovement.setEnded(true);
        }

        this.currentMovement = null;
        this.segment = null;
        this.targetHeading = null;
        this.targetTurnPoseVec = null;
        this.turnDirection = 0.0;
        this.turnTotalDisplacement = 0.0;

        this.drivetrain.stop();
    }

    /**
     * Halts the current movement temporarily without clearing the target state.
     */
    public void pause() {
        this.paused = true;
        this.drivetrain.stop();
    }

    /**
     * Resumes a paused movement from the robots current location.
     */
    public void resume() {
        if (this.paused) {
            this.paused = false;
            this.lastNano = -1;
        }
    }

    /**
     * Drives the robot using joystick inputs adjusted for field centric control.
     * Stops any active autonomous movement.
     *
     * @param x            The left and right strafe input positive for right.
     * @param y            The forward and backward drive input positive for forward.
     * @param turn         The rotation input positive for counterclockwise.
     * @param robotHeading The current heading of the robot in radians.
     */
    public void teleOpDrive(double x, double y, double turn, double robotHeading) {
        if (isBusy()) {stop();}
        drivetrain.drive(x, y, turn, robotHeading);
    }

    /**
     * Drives the robot using joystick inputs for standard robot centric control.
     * Stops any active autonomous movement.
     *
     * @param x    The left and right strafe input positive for right.
     * @param y    The forward and backward drive input positive for forward.
     * @param turn The rotation input positive for counterclockwise.
     */
    public void teleOpDrive(double x, double y, double turn) {teleOpDrive(x, y, turn, 0);}

    /**
     * Retrieves the robots current pose estimate from the localizer.
     *
     * @return The current global position and heading.
     */
    public Pose getPose() {return localizer.getPose();}

    /**
     * Checks if the follower is currently executing a movement.
     *
     * @return true if a movement is in progress false otherwise.
     */
    public boolean isBusy() {
        return currentMovement != null;
    }

    /**
     * Forcibly overrides the localizers current pose estimate.
     *
     * @param pose The new global position and heading.
     */
    public void setPose(Pose pose) {localizer.setPose(pose);}

    /**
     * Retrieves the robots current velocity estimate from the localizer.
     *
     * @return The current velocity expressed in robot local frame.
     */
    public Pose getVelocity() {return localizer.getVel();}

    /**
     * Retrieves the robots current acceleration estimate from the localizer.
     *
     * @return The current acceleration expressed in robot local frame.
     */
    public Pose getAcceleration() {return localizer.getAccel();}
}
