package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import tuning.TunerContext;
import tuning.TuningPhase;
import tuning.AngularCharacterizationPhase;
import tuning.ForwardCharacterizationPhase;
import tuning.PreflightPhase;
import tuning.StrafeCharacterizationPhase;

import core.Follower;
import geometry.Pose;

/**
 * Unified follower tuning class for Apex Pathing
 *
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    private final TunerContext context = new TunerContext(this);

    private String[] phaseSelectorStatuses;
    private int selectedPhaseIndex = 0;
    private TuningPhase runningPhase = null;

    enum Phase { PREFLIGHT, FORWARD_CHARACTERIZATION, STRAFE_CHARACTERIZATION,
        ANGULAR_CHARACTERIZATION }
    private static final Phase[] phases = {
            Phase.PREFLIGHT,
            Phase.FORWARD_CHARACTERIZATION,
            Phase.STRAFE_CHARACTERIZATION,
            Phase.ANGULAR_CHARACTERIZATION
    };

    @Override
    public void runOpMode() {
        context.setFollower(new Follower(new Constants(), hardwareMap));
        resetPhaseSelectionData(); // Initialize the phase selector statuses

        while (opModeInInit() && runningPhase == null) {
            TuningPhase selectedPhase = phaseSelector(); // Will remain null until a phase is selected
            if (selectedPhase != null) {
                runningPhase = selectedPhase;
            }
        }

        if (runningPhase != null) {
            context.getFollower().setPose(Pose.zero());
            telemetry.addLine("Press the start button to run the tuner");
            telemetry.addLine("Make sure you have adequate space to run the robot safely!");
            telemetry.update();
        } else {
            requestOpModeStop(); // The OpMode was started without a phase, so we stop it
        }

        waitForStart();

        while (opModeIsActive()) {
            if (runningPhase == null) {
                continue; // OpMode stop was requested already, do nothing
            }
            telemetry.clearAll();
            context.getFollower().update();
            boolean complete = runningPhase.update(gamepad1.aWasPressed(), gamepad1.bWasPressed());
            telemetry.update();
            if (complete) {
                runningPhase = null;
                context.constants.drivetrainType = context.getFollower().getDrivetrain()
                        .getDrivetrainType();
                context.saveConstants();
                requestOpModeStop();
            }
        }
    }

    private void resetPhaseSelectionData() {
        boolean forwardIsTuned = context.constants.translationalKV > 0.0 &&
                context.constants.translationalKA > 0.0 &&
                context.constants.forwardVelLimitIn > 0.0;
        boolean strafeIsTuned = context.constants.lateralCoeffs.kP > 0.0 &&
                context.constants.strafeVelLimitIn > 0.0;
        boolean angularIsTuned = context.constants.angularKV > 0.0 &&
                context.constants.angularKA > 0.0 &&
                context.constants.angularVelLimitRad > 0.0;
        boolean holonomic = context.getFollower().getDrivetrain().isHolonomic();

        this.phaseSelectorStatuses = new String[]{
                "[ ]",
                forwardIsTuned ? "[✓]" : "[ ]",
                holonomic ? (strafeIsTuned ? "[✓]" : "[ ]") : "[N/A]",
                angularIsTuned ? "[✓]" : "[ ]"
        };

        this.selectedPhaseIndex = 0;
        for (int i = 0; i < phaseSelectorStatuses.length; i++) {
            if (phaseSelectorStatuses[i].equals("[ ]")) {
                this.selectedPhaseIndex = i;
                break;
            }
        }
    }

    private TuningPhase phaseSelector() {
        telemetry.addLine(
                "The Apex Pathing tuners are listed in order of execution below."
        );
        telemetry.addLine(
                "[✓] = Already Tuned (you can still select to retune)," +
                        "[N/A] = Not applicable to this drivetrain," +
                        "[ ] = Next tuner to run. The cursor ('<') is here by default."
        );
        telemetry.addLine("Use the DPad Up and Down buttons to cycle through phases, " +
                "then press B to open the selected phase.");
        telemetry.addLine();

        for (int i = 0; i < phases.length; i++) {
            String cursor = (i == selectedPhaseIndex) ? " <" : "";
            telemetry.addLine(phaseSelectorStatuses[i] + " " +
                    phases[i].toString().replace("_", " ") + cursor);
        }
        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            selectedPhaseIndex = (selectedPhaseIndex - 1 + phases.length) % phases.length;
            // Don't allow selection of unavailable phases
            while (!isSelectable(selectedPhaseIndex)) {
                selectedPhaseIndex = (selectedPhaseIndex - 1 + phases.length) % phases.length;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            selectedPhaseIndex = (selectedPhaseIndex + 1) % phases.length;
            while (!isSelectable(selectedPhaseIndex)) {
                selectedPhaseIndex = (selectedPhaseIndex + 1) % phases.length;
            }
        } else if (gamepad1.bWasPressed()) {
            Phase selectedPhase = phases[selectedPhaseIndex];
            switch (selectedPhase) {
                case PREFLIGHT:
                    return new PreflightPhase(context);
                case FORWARD_CHARACTERIZATION:
                    return new ForwardCharacterizationPhase(context);
                case STRAFE_CHARACTERIZATION:
                    return new StrafeCharacterizationPhase(context);
                case ANGULAR_CHARACTERIZATION:
                    return new AngularCharacterizationPhase(context);
            }
        }

        return null; // Phase hasn't been selected yet
    }

    private boolean isSelectable(int index) {
        return !phaseSelectorStatuses[index].equals("[N/A]");
    }
}
