package feedforward;

public class MotionParameters {
    private double tangentialVel;
    private double tangentialAccel;
    private double angularVel;
    private double angularAccel;
    private double distAlongCurve = 0.0;
    private double motorPower = 0.0;

    /**
     * No-args constructor to initialize a blank, reusable parameters object.
     */
    public MotionParameters() {
        this.tangentialVel = 0.0;
        this.tangentialAccel = 0.0;
        this.angularVel = 0.0;
        this.angularAccel = 0.0;
        this.distAlongCurve = 0.0;
    }

    /**
     * All-args constructor for immediate full initialization.
     */
    public MotionParameters(double tangentialVel, double tangentialAccel, double angularVel,
                            double angularAccel, double distAlongCurve) {
        this.tangentialVel = tangentialVel;
        this.tangentialAccel = tangentialAccel;
        this.angularVel = angularVel;
        this.angularAccel = angularAccel;
        this.distAlongCurve = distAlongCurve;
    }

    public MotionParameters(double tangentialVel, double tangentialAccel, double angularVel,
                            double angularAccel) {
        this.tangentialVel = tangentialVel;
        this.tangentialAccel = tangentialAccel;
        this.angularVel = angularVel;
        this.angularAccel = angularAccel;
    }

    public MotionParameters setTangentialVel(double tangentialVel) {
        this.tangentialVel = tangentialVel;
        return this;
    }

    public MotionParameters setTangentialAccel(double tangentialAccel) {
        this.tangentialAccel = tangentialAccel;
        return this;
    }

    public MotionParameters setAngularVel(double angularVel) {
        this.angularVel = angularVel;
        return this;
    }

    public MotionParameters setAngularAccel(double angularAccel) {
        this.angularAccel = angularAccel;
        return this;
    }

    public double getMotorPower() { return motorPower; }

    public void setMotorPower(double motorPower) { this.motorPower = motorPower; }

    public void setDistAlongCurve(double distAlongCurve) {
        this.distAlongCurve = distAlongCurve;
    }

    public double getTangentialVel() {
        return tangentialVel;
    }

    public double getTangentialAccel() {
        return tangentialAccel;
    }

    public double getAngularVel() {
        return angularVel;
    }

    public double getAngularAccel() {
        return angularAccel;
    }

    public double getDistAlongCurve() {
        return distAlongCurve;
    }

    public double getProgression() {
        return distAlongCurve;
    }
}