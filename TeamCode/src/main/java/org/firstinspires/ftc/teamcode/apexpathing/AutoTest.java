package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import core.Follower;
import paths.ExampleAutoPath;
import util.PoseFactory;

/**
 * Test Auto utilizing {@link ExampleAutoPath}
 * IMPORTANT: Make sure your {@link core.FollowerConstants} have been tuned by running {@link FollowerTuner} before running this OpMode
 * @author Sohum Arora 22985 Paraducks
 */
@Autonomous(name = "Apex Auto Test", group = "Apex Pathing Tests")
public class AutoTest extends LinearOpMode {
    Constants constants = new Constants();
    ExampleAutoPath path = new ExampleAutoPath(PoseFactory.Mirror.NONE);
    enum Path {TEST_PATH, TEST_TURN, COMPLETE}
    Path currentState = Path.TEST_PATH;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(constants, hardwareMap);

        while (opModeInInit()){
            telemetry.addLine("Robot initialized");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            switch (currentState) {
                case TEST_PATH:
                    if (!path.testPath.hasStarted()) follower.follow(path.testPath);
                    if (path.testPath.hasEnded()) currentState = Path.TEST_TURN;
                    break;

                case TEST_TURN:
                    if (!path.testTurn.hasStarted()) follower.follow(path.testTurn);
                    if (path.testTurn.hasEnded()) currentState = Path.COMPLETE;
                    break;

                case COMPLETE:
                    telemetry.addLine("Auto Test Completed!");
                    break;
            }

            telemetry.addLine(follower.isBusy() ? "Follower IS busy" : "Follower is NOT busy");
            telemetry.addData("Current X", follower.getPose().getX());
            telemetry.addData("Current Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}