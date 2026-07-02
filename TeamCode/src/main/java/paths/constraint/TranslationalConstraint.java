package paths.constraint;

import geometry.Dist;

public class TranslationalConstraint implements PathConstraint {
    private double s;
    private final ConstraintType type;
    private final double value_in;

    public TranslationalConstraint(double s, ConstraintType type, Dist value) {
        this.s = s;
        this.type = type;
        this.value_in = value.getIn();
    }

    @Override
    public double getS() {
        return s;
    }

    @Override
    public void setS(double s) {
        this.s = s;
    }

    @Override
    public ConstraintType getType() {
        return type;
    }

    public double getValueIn() {
        return value_in;
    }
}