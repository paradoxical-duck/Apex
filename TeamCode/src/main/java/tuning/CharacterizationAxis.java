package tuning;

import geometry.Pose;

/** Robot axes characterized independently by the tuner. */
public enum CharacterizationAxis {
    FORWARD("Forward", 0.50, 60.0),
    STRAFE("Strafe", 0.50, 60.0),
    ANGULAR("Angular", 0.04, Math.PI * 1.5);

    private final String displayName;
    private final double minimumSampleVelocity;
    private final double maximumTestTravel;

    CharacterizationAxis(String displayName, double minimumSampleVelocity,
                         double maximumTestTravel) {
        this.displayName = displayName;
        this.minimumSampleVelocity = minimumSampleVelocity;
        this.maximumTestTravel = maximumTestTravel;
    }

    public String displayName() { return displayName; }

    public double minimumSampleVelocity() { return minimumSampleVelocity; }

    public double maximumTestTravel() { return maximumTestTravel; }

    public double position(Pose pose) {
        switch (this) {
            case FORWARD: return pose.getX().getIn();
            case STRAFE: return pose.getY().getIn();
            case ANGULAR: return pose.getHeading().getRad();
            default: throw new IllegalStateException("Unhandled characterization axis");
        }
    }

    public double velocity(Pose pose) { return position(pose); }
}
