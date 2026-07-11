package core;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrain;

/**
 * Class to hold constants for the Follower class. These constants are loaded from a JSON file
 * created by the tuners. If the file does not exist or cannot be read, default values will be used.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author DrPixelCat
 */
public class FollowerConstants {
    private static FollowerConstants instance;
    /* Note to developers:
    If you want to add new constants, create the variable here and add it to the loadValues() and
    toJson() methods. This will ensure that the new constants are loaded from the JSON file and
    saved back to it. */
    public BaseDrivetrain.DrivetrainType drivetrainType =
            BaseDrivetrain.DrivetrainType.MECANUM;
    public PDSCoefficients headingCoeffs = new PDSCoefficients();
    public PDSCoefficients translationalCoeffs = new PDSCoefficients();
    public PDSCoefficients lateralCoeffs = new PDSCoefficients();

    public double velocityFeedbackGain = 0.0;
    public double angularVelocityFeedbackGain = 0.0;
    public double translationalKV = 0.0, translationalKA = 0.0;
    public double angularKV = 0.0, angularKA = 0.0;
    public double Kcentripetal = 0.0;

    public double forwardVelLimitIn = 0.0;
    public double forwardAccelLimitIn = 0.0;
    public double strafeVelLimitIn = 0.0;
    public double strafeAccelLimitIn = 0.0;
    public double angularVelLimitRad = 0.0;
    public double angularAccelLimitRad = 0.0;

    private FollowerConstants() { reload(); }

    public static FollowerConstants getInstance() {
        if (instance == null) {
            instance = new FollowerConstants();
        }
        return instance;
    }

    private double loadDouble(JSONObject json, String key, double fallback) {
        double value = json.optDouble(key, fallback);
        return Double.isFinite(value) ? value : fallback;
    }

    public void reload() {
        File file = new File(
                Environment.getExternalStorageDirectory().getPath() +
                        "/FIRST/ApexPathing/constants.json"
        );
        if (!file.exists()) return;

        JSONObject json;
        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) sb.append(line);
            reader.close();
            json = new JSONObject(sb.toString());
        } catch (Exception e) {
            return;
        }

        try {
            drivetrainType = BaseDrivetrain.DrivetrainType.valueOf(
                    json.optString("drivetrainType", drivetrainType.toString())
            );
        } catch (IllegalArgumentException ignored) {
            // Preserve the safe in-code default when an old/corrupt JSON value is encountered.
        }

        headingCoeffs.setkP(loadDouble(json, "headingP", headingCoeffs.kP));
        headingCoeffs.setkD(loadDouble(json, "headingD", headingCoeffs.kD));
        headingCoeffs.setkS(loadDouble(json, "headingS", headingCoeffs.kS));

        translationalCoeffs.setkP(loadDouble(json, "translationalP", translationalCoeffs.kP));
        translationalCoeffs.setkD(loadDouble(json, "translationalD", translationalCoeffs.kD));
        translationalCoeffs.setkS(loadDouble(json, "translationalS", translationalCoeffs.kS));

        // Old files used one controller for both axes. Preserve that as the migration fallback.
        lateralCoeffs.setkP(loadDouble(json, "lateralP", translationalCoeffs.kP));
        lateralCoeffs.setkD(loadDouble(json, "lateralD", translationalCoeffs.kD));
        lateralCoeffs.setkS(loadDouble(json, "lateralS", translationalCoeffs.kS));

        translationalKV = loadDouble(json, "translationKV", translationalKV);
        translationalKA = loadDouble(json, "translationKA", translationalKA);
        angularKV = loadDouble(json, "angularKV", angularKV);
        angularKA = loadDouble(json, "angularKA", angularKA);
        velocityFeedbackGain = loadDouble(json, "velocityFeedbackGain", velocityFeedbackGain);
        angularVelocityFeedbackGain = loadDouble(json, "angularVelocityFeedbackGain",
                angularVelocityFeedbackGain);
        Kcentripetal = loadDouble(json, "Kcentripetal", Kcentripetal);

        forwardVelLimitIn = loadDouble(json, "forwardVelLimitIn", forwardVelLimitIn);
        forwardAccelLimitIn = loadDouble(json, "forwardAccelLimitIn", forwardAccelLimitIn);
        strafeVelLimitIn = loadDouble(json, "strafeVelLimitIn", strafeVelLimitIn);
        strafeAccelLimitIn = loadDouble(json, "strafeAccelLimitIn", strafeAccelLimitIn);
        angularVelLimitRad = loadDouble(json, "angularVelLimitRad", angularVelLimitRad);
        angularAccelLimitRad = loadDouble(json, "angularAccelLimitRad", angularAccelLimitRad);
    }

    public JSONObject toJson() {
        JSONObject json = new JSONObject();
        try {
            json.put("drivetrainType", drivetrainType.toString());
            json.put("headingP", headingCoeffs.kP);
            json.put("headingD", headingCoeffs.kD);
            json.put("headingS", headingCoeffs.kS);
            json.put("translationalP", translationalCoeffs.kP);
            json.put("translationalD", translationalCoeffs.kD);
            json.put("translationalS", translationalCoeffs.kS);
            json.put("lateralP", lateralCoeffs.kP);
            json.put("lateralD", lateralCoeffs.kD);
            json.put("lateralS", lateralCoeffs.kS);
            json.put("translationKV", translationalKV);
            json.put("translationKA", translationalKA);
            json.put("angularKV", angularKV);
            json.put("angularKA", angularKA);
            json.put("velocityFeedbackGain", velocityFeedbackGain);
            json.put("angularVelocityFeedbackGain", angularVelocityFeedbackGain);
            json.put("Kcentripetal", Kcentripetal);
            json.put("forwardVelLimitIn", forwardVelLimitIn);
            json.put("forwardAccelLimitIn", forwardAccelLimitIn);
            json.put("strafeVelLimitIn", strafeVelLimitIn);
            json.put("strafeAccelLimitIn", strafeAccelLimitIn);
            json.put("angularVelLimitRad", angularVelLimitRad);
            json.put("angularAccelLimitRad", angularAccelLimitRad);
        } catch (Exception ignored) {
            // JSONObject only rejects unsupported values; all fields above are primitives.
        }
        return json;
    }
}
