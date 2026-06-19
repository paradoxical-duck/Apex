package core;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;
import org.json.JSONObject;
import java.io.File;
import java.io.BufferedReader;
import java.io.FileReader;

/**
 * Apex Pathing FollowerConstants class
 * Internally assigns the coefficient values determined through tuning directly,
 * thereby eliminating the need to manually tune and set the values in the Constants file!
 * @author Sohum Arora 22985 Paraducks
 */
public class FollowerConstants {
    public PDSCoefficients headingCoeffs = new PDSCoefficients();
    public PDSCoefficients lateralCoeffs = new PDSCoefficients();
    public PDSCoefficients driveCoeffs = new PDSCoefficients();
    public PDSCoefficients velocityCoeffs = new PDSCoefficients();
    public double lateralKV = 0.0, lateralKA = 0.0;
    public double angularKV = 0.0, angularKA = 0.0;
    public Dist velocityLimit = null;
    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);
    public double tTolerance = 0.95;
    public double maxLateralAccel = 40.0;

    public FollowerConstants() {
        loadValues();
    }

    private void loadValues() {
        File file = new File("/sdcard/FIRST/FollowerConstants.json");
        if (file.exists()) {
            try {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) sb.append(line);
                reader.close();

                JSONObject json = new JSONObject(sb.toString());

                this.headingCoeffs = new PDSCoefficients(
                        json.optDouble("headingP", 0),
                        json.optDouble("headingD", 0),
                        json.optDouble("headingS", 0), 0);

                double tP = json.optDouble("translationP", 0);
                double tD = json.optDouble("translationD", 0);
                double tS = json.optDouble("translationS", 0);
                this.driveCoeffs = new PDSCoefficients(tP, tD, tS, 0);
                this.lateralCoeffs = new PDSCoefficients(tP, tD, tS, 0);

                this.lateralKV = json.optDouble("translationKV", this.lateralKV);
                this.lateralKA = json.optDouble("translationKA", this.lateralKA);
                this.angularKV = json.optDouble("angularKV", this.angularKV);
                this.angularKA = json.optDouble("angularKA", this.angularKA);
                this.maxLateralAccel = json.optDouble("maxLateralAccel", this.maxLateralAccel);
                this.headingTolerance = Angle.fromDeg(json.optDouble("headingToleranceDeg", 1.0));
                this.distanceTolerance = Dist.fromIn(json.optDouble("distanceToleranceIn", 0.5));
                this.tTolerance = json.optDouble("tTolerance", 0.95);

            } catch (Exception ignored) {
                //defaults to 0 values everywhere
            }
        }
    }
    public FollowerConstants getConstants() {
        return this;
    }
}