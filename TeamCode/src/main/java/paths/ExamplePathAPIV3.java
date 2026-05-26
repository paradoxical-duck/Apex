package paths;

import paths.heading.InterpolationStyle;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseFactory;

public class ExamplePathAPIV3 {
    private Distance.Units distUnit = Distance.Units.INCHES;
    private Angle.Units angleUnit = Angle.Units.DEGREES;
    public PoseFactory pose = new PoseFactory(distUnit, angleUnit);
    private Pose startPose;

    public ExamplePathAPIV3(boolean mirror) {
        pose.setMirror(mirror);
        // Uses the builder's build method to apply units and mirroring
        startPose = pose.build(0, 0, 0);
    }

    public void exampleCallback() {
        // This will run when the follower reaches 50% of the path
    }

    public BSplinePath testPath() {
        return new BSplinePathBuilder(startPose)
                .addControlPoints(
                        pose.at(10, 0),
                        pose.at(0, 0),
                        pose.filletAt(15, 30, 10),
                        pose.at(30, 0, 40)
                )
                // TODO: make this function! Use s instead of t for better user experience
//                .addCallback(0.5, () -> {
//                    exampleCallback();
//                })
                .interpolateWith(InterpolationStyle.TANGENT_OPTIMAL)
                .build();
    }
}