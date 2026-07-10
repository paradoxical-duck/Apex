package core;

import drivetrains.BaseDrivetrainConstants;
import localizers.BaseLocalizerConstants;

/**
 * Abstract base class for your constants
 * Methods implemented by your Constants file
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Sohum Arora 22985 Paraducks
 */
public abstract class ApexConfig {
    public abstract BaseDrivetrainConstants<?> drivetrainConfig();

    public abstract BaseLocalizerConstants<?> localizerConfig();

    public abstract FollowerConstants followerConfig();
}