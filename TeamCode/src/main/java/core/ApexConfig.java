package core;

import drivetrains.BaseDrivetrainConfig;
import localizers.BaseLocalizerConfig;

// TODO: Rewrite JavaDocs
/**
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class ApexConfig {
    public abstract BaseDrivetrainConfig<?> drivetrainConfig();

    public abstract BaseLocalizerConfig<?> localizerConfig();

    public abstract FollowerConfig followerConfig();
}