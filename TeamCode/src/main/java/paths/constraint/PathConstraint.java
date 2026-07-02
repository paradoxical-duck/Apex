package paths.constraint;

/**
 * A marker interface for all path-based kinematic constraints.
 */
public interface PathConstraint {
    
    /**
     * @return The percentage along the path [0.0, 1.0] where this constraint becomes active.
     */
    double getS();

    /**
     * Set's the s percentage along the path [0.0, 1.0]
     */
    void setS(double s);

    /**
     * @return The type of path constraint: VELOCITY, or ACCELERATION
     */
    ConstraintType getType();
}