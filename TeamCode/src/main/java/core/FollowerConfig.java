package core;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;

/**
 * Apex Pathing Follower configuration class.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class FollowerConfig {
    public PDSCoefficients headingCoeffs = new PDSCoefficients();
    public PDSCoefficients lateralCoeffs = new PDSCoefficients();
    public PDSCoefficients driveCoeffs = new PDSCoefficients();
    public PDSCoefficients velocityCoeffs = new PDSCoefficients();

    public double headingKV, headingKA = 0.0;
    public double lateralKV, lateralKA = 0.0;
    public Dist velocityLimit = null;

    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);
    public double tTolerance = 0.95;
    public double maxLateralAccel = 0;

    /** Set the heading controller PDS coefficients. By default, all values are zero. */
    public FollowerConfig setHeadingCoeffs(PDSCoefficients headingCoeffs) {
        this.headingCoeffs = headingCoeffs; return this;
    }

    /** Set the lateral controller PDS coefficients. By default, all values are zero. */
    public FollowerConfig setLateralCoeffs(PDSCoefficients lateralCoeffs) {
        this.lateralCoeffs = lateralCoeffs; return this;
    }

    /** Set the drive controller PDS coefficients. By default, all values are zero. */
    public FollowerConfig setDriveCoeffs(PDSCoefficients driveCoeffs) {
        this.driveCoeffs = driveCoeffs; return this;
    }

    /** Set the velocity controller PDS coefficients. By default, all values are zero. */
    public FollowerConfig setVelocityCoeffs(PDSCoefficients velocityCoeffs) {
        this.velocityCoeffs = velocityCoeffs; return this;
    }

    /** Set the heading feedforward velocity and acceleration coefficients. By default, both values are zero. */
    public FollowerConfig setFeedforwardCoeffs(double headingKV, double headingKA) {
        this.headingKV = headingKV; this.headingKA = headingKA; return this;
    }

    /** Set the lateral feedforward velocity and acceleration coefficients. By default, both values are zero. */
    public FollowerConfig setLateralFeedforwardCoeffs(double kV, double kA) {
        this.lateralKV = kV; this.lateralKA = kA; return this;
    }

    /** Set the velocity limit for the follower. By default, there is no limit. */
    public FollowerConfig setVelocityLimit(Dist velocityLimit) {
        this.velocityLimit = velocityLimit; return this;
    }

    /** Set the heading tolerance for the follower. By default, it is 1 degree. */
    public FollowerConfig setHeadingTolerance(Angle headingTolerance) {
        this.headingTolerance = headingTolerance; return this;
    }

    /** Set the distance tolerance for the follower. By default, it is 0.5 inches. */
    public FollowerConfig setDistanceTolerance(Dist distanceTolerance) {
        this.distanceTolerance = distanceTolerance; return this;
    }

    /** Set the t tolerance for the follower. By default, it is 0.95. */
    public FollowerConfig setTTolerance(double tTolerance) {
        this.tTolerance = tTolerance; return this;
    }

    /** Set the maximum lateral acceleration for the follower. By default, there is no limit. */
    public FollowerConfig setMaxLateralAccel(double maxLateralAccel) {
        this.maxLateralAccel = maxLateralAccel; return this;
    }
}