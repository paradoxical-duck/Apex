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
    public double translationalKV = 0.0;
    public double translationalKA = 0.0;
    public double angularKV = 0.0;
    public double angularKA = 0.0;
    public double Kcentripetal = 0.0;

    public double forwardVelLimitIn = 0.0;
    public double forwardAccelLimitIn = 0.0;
    public double strafeVelLimitIn = 0.0;
    public double strafeAccelLimitIn = 0.0;
    public double angularVelLimitRad = 0.0;
    public double angularAccelLimitRad = 0.0;

    private FollowerConstants() {
        reload();
    }

    public static FollowerConstants getInstance() {
        if (instance == null) {
            instance = new FollowerConstants();
        }
        return instance;
    }

    public void reload() {
        File file = new File(
                Environment.getExternalStorageDirectory().getPath()
                        + "/FIRST/ApexPathing/constants.json"
        );

        if (!file.exists()) {
            return;
        }

        JSONObject json;

        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            StringBuilder sb = new StringBuilder();
            String line;

            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }

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
        }

        headingCoeffs.setkP(json.optDouble("headingP", 0.0));
        headingCoeffs.setkD(json.optDouble("headingD", 0.0));
        headingCoeffs.setkS(json.optDouble("headingS", 0.0));

        translationalCoeffs.setkP(json.optDouble("translationalP", 0.0));
        translationalCoeffs.setkD(json.optDouble("translationalD", 0.0));
        translationalCoeffs.setkS(json.optDouble("translationalS", 0.0));

        lateralCoeffs.setkP(
                json.optDouble("lateralP", translationalCoeffs.kP)
        );
        lateralCoeffs.setkD(
                json.optDouble("lateralD", translationalCoeffs.kD)
        );
        lateralCoeffs.setkS(
                json.optDouble("lateralS", translationalCoeffs.kS)
        );

        translationalKV = json.optDouble("translationKV", 0.0);
        translationalKA = json.optDouble("translationKA", 0.0);
        angularKV = json.optDouble("angularKV", 0.0);
        angularKA = json.optDouble("angularKA", 0.0);

        velocityFeedbackGain =
                json.optDouble("velocityFeedbackGain", 0.0);

        angularVelocityFeedbackGain =
                json.optDouble("angularVelocityFeedbackGain", 0.0);

        Kcentripetal = json.optDouble("Kcentripetal", 0.0);

        forwardVelLimitIn =
                json.optDouble("forwardVelLimitIn", 0.0);

        forwardAccelLimitIn =
                json.optDouble("forwardAccelLimitIn", 0.0);

        strafeVelLimitIn =
                json.optDouble("strafeVelLimitIn", 0.0);

        strafeAccelLimitIn =
                json.optDouble("strafeAccelLimitIn", 0.0);

        angularVelLimitRad =
                json.optDouble("angularVelLimitRad", 0.0);

        angularAccelLimitRad =
                json.optDouble("angularAccelLimitRad", 0.0);
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
            json.put(
                    "angularVelocityFeedbackGain",
                    angularVelocityFeedbackGain
            );

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
