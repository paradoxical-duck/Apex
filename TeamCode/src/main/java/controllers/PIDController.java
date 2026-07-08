package controllers;

/**
 * A general purpose PID controller specifically made for controlling a robot.
 * <ul>
 * <li><b>kP</b> - proportional gain (how aggressively the controller responds to error)</li>
 * <li><b>kI</b> - integral gain (how much the controller accounts for accumulated past error)</li>
 * <li><b>kD</b> - derivative gain (how much the controller accounts for error rate of change)</li>
 * </ul>
 *
 * @author DrPixelCat
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class PIDController extends Controller {
    private PIDCoefficients coeffs;
    private double integralSum = 0.0;

    // region Coefficients class

    /**
     * A simple class to hold the PIDCoefficients for the PIDController. This makes it easier
     * to set and manage the PIDCoefficients.
     */
    public static class PIDCoefficients {
        public double kP, kI, kD;

        /**
         * Creates a PIDCoefficients object with the given values. Check the
         * {@link PIDController controller} documentation for what each coefficient does.
         */
        public PIDCoefficients(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        /**
         * Creates a PIDCoefficients object with the given P and D values, with the I term
         * set to 0. Check the {@link PIDController controller} documentation for what each
         * coefficient
         * does.
         */
        public PIDCoefficients(double kP, double kD) {
            this(kP, 0.0, kD);
        }

        /**
         * Creates a PIDCoefficients object with all coefficients set to 0.
         */
        public PIDCoefficients() {
            this(0.0, 0.0, 0.0);
        }
    }

    // region Constructors and getters/setters

    /**
     * Creates a PIDController.
     *
     * @param coefficients the {@link PIDCoefficients} to use for the controller
     */
    public PIDController(PIDCoefficients coefficients) {
        this.setCoefficients(coefficients);
    }

    /**
     * @param coefficients the {@link PIDCoefficients} to use for the controller
     */
    public void setCoefficients(PIDCoefficients coefficients) {
        this.coeffs = coefficients;
    }

    /**
     * @return the current {@link PIDCoefficients} being used by the controller
     */
    public PIDCoefficients getCoefficients() {
        return this.coeffs;
    }

    /**
     * Resets the accumulated integral sum to zero.
     * Call this when starting a new movement to prevent integral windup.
     */
    public void resetIntegral() {
        this.integralSum = 0.0;
    }

    @Override
    protected double computeOutput(double error, double lastError, double deltaTime) {
        // Calculate Proportional
        double proportional = this.coeffs.kP * error;

        // Calculate Integral (only accumulate if there's no time anomaly)
        if (!timeAnomalyDetected) {
            this.integralSum += error * deltaTime;
        }
        double integral = this.coeffs.kI * this.integralSum;

        // Calculate Derivative
        double derivative = this.coeffs.kD * (timeAnomalyDetected ? 0.0 :
                (error - lastError) / deltaTime);

        return proportional + integral + derivative;
    }
}