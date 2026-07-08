package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import util.MotorFactory;

/**
 * Drivetrain class that physically actuates between holonomic and high-traction kinematics.
 * Supports Butterfly, Locking Mecanum, or similar drivetrains.
 *
 * @author DrPixelCat - 7842 Alum
 */
public class DualActuated extends BaseDrivetrain<DualActuated.Config> {

    public enum DriveState {
        TANK,  // Locked rollers or deployed traction wheels (Tank kinematics)
        HOLONOMIC  // Unlocked rollers or retracted traction wheels (Mecanum kinematics)
    }

    private DriveState state;
    private final List<Actuator> actuators = new ArrayList<>();

    public DualActuated(Config config, HardwareMap hardwareMap) {
        super(config, hardwareMap);

        if (Objects.equals(config.blMotorConfig, null) || Objects.equals(config.brMotorConfig,
                null)) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for an Actuated " +
                            "Dual Drivetrain."
            );
        }

        for (Config.ServoConfig sc : config.servoConfigs) {
            Servo servo = hardwareMap.get(Servo.class, sc.name);
            actuators.add(new Actuator(servo, sc.tractionPos, sc.holonomicPos));
        }

        // Apply the initial state defined in the configuration
        forceApplyState(config.initialState);
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        if (state == DriveState.TANK) {
            setPowers(x - turn, x + turn,
                    x - turn, x + turn);
        } else {
            setPowers(x - y - turn, x + y + turn,
                    x + y - turn, x - y + turn
            );
        }
    }

    @Override
    public boolean isHolonomic() {
        return state != DriveState.TANK;
    }

    /**
     * Sets the configuration to the TRACTION state
     */
    public void activateTractionState() {
        if (this.state != DriveState.TANK) {
            forceApplyState(DriveState.TANK);
        }
    }

    /**
     * Sets the configuration to the HOLONOMIC state
     */
    public void activateHolonomicState() {
        if (this.state != DriveState.HOLONOMIC) {
            forceApplyState(DriveState.HOLONOMIC);
        }
    }

    public DriveState getDriveState() {
        return state;
    }

    private void forceApplyState(DriveState newState) {
        this.state = newState;
        for (Actuator actuator : actuators) {
            actuator.servo.setPosition(state == DriveState.TANK ? actuator.tankPos :
                    actuator.holonomicPos);
        }
        applyStateLimits();
    }

    /**
     * Copies the tuning limits from the active sub-configuration into the root config.
     * This ensures BaseDrivetrain applies the correct constraints.
     */
    private void applyStateLimits() {
        BaseDrivetrainConfig<?> activeConfig = (state == DriveState.TANK) ?
                config.tractionConfig : config.holonomicConfig;

        this.config.maxPower = activeConfig.maxPower;
        this.config.linearVelocityLimit = activeConfig.linearVelocityLimit;
        this.config.angularVelocityLimit = activeConfig.angularVelocityLimit;
        this.config.linearAccelerationLimit = activeConfig.linearAccelerationLimit;
        this.config.angularAccelerationLimit = activeConfig.angularAccelerationLimit;
        this.config.robotCentric = activeConfig.robotCentric;
    }

    /**
     * Helper class to create servo and store states
     */
    private static class Actuator {
        final Servo servo;
        final double tankPos;
        final double holonomicPos;

        Actuator(Servo servo, double tankPos, double holonomicPos) {
            this.servo = servo;
            this.tankPos = tankPos;
            this.holonomicPos = holonomicPos;
        }
    }

    /**
     * Configuration class for an Actuated Dual Drivetrain.
     */
    public static class Config extends BaseDrivetrainConfig<Config> {
        // Sub-configurations for behavioral limits
        public BaseDrivetrainConfig<?> holonomicConfig = new Mecanum.Config();
        public BaseDrivetrainConfig<?> tractionConfig = new Tank.Config();

        // Define the initial startup state (defaults to Holonomic)
        public DriveState initialState = DriveState.HOLONOMIC;

        public final List<ServoConfig> servoConfigs = new ArrayList<>();

        @Override
        public DualActuated build(HardwareMap hardwareMap) {
            return new DualActuated(this, hardwareMap);
        }

        /**
         * Sets the initial startup state of the drivetrain
         */
        public Config setInitialState(DriveState initialState) {
            this.initialState = initialState;
            return this;
        }

        /**
         * Sets the stored configuration for the Mecanum state.
         */
        public Config setHolonomicConfig(BaseDrivetrainConfig<?> holonomicConfig) {
            this.holonomicConfig = holonomicConfig;
            return this;
        }

        /**
         * Sets the stored configuration for the Traction (locked/Tank) state.
         */
        public Config setTractionConfig(BaseDrivetrainConfig<?> tractionConfig) {
            this.tractionConfig = tractionConfig;
            return this;
        }

        public Config setFrontLeftMotor(MotorFactory motorFactory) {
            this.flMotorConfig = motorFactory;
            return this;
        }

        public Config setFrontRightMotor(MotorFactory motorFactory) {
            this.frMotorConfig = motorFactory;
            return this;
        }

        public Config setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        public Config setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }

        /**
         * Adds an actuation servo to the drivetrain configuration.
         * * @param name The hardware map name of the servo
         *
         * @param tractionPosition  The physical servo position for the high-traction state (e.g.
         *                          locked or wheel deployed)
         * @param holonomicPosition The physical servo position for the holonomic state (e.g.
         *                          unlocked or wheel retracted)
         */
        public Config addActuator(String name, double tractionPosition, double holonomicPosition) {
            servoConfigs.add(new ServoConfig(name, tractionPosition, holonomicPosition));
            return this;
        }

        public static class ServoConfig {
            public final String name;
            public final double tractionPos;
            public final double holonomicPos;

            public ServoConfig(String name, double tractionPos, double holonomicPos) {
                this.name = name;
                this.tractionPos = tractionPos;
                this.holonomicPos = holonomicPos;
            }
        }
    }
}