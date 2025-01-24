package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BasicAutoOp")
public class BasicAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(34.6, 63.3, Math.toRadians(-90)));

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(-64.0, -48.0, Math.toRadians(180.0));
        drive.setPoseEstimate(startPose);

        // Create a trajectory to move forward
        Trajectory trajectoryForward = drive.trajectoryBuilder(startPose)
                .forward(48.0)
                .build();

        // Create a trajectory to strafe
        Trajectory trajectoryStrafe = drive.trajectoryBuilder(trajectoryForward.end())
                .strafeRight(24.0)
                .build();

        // Create a trajectory to spline
        Trajectory trajectorySpline = drive.trajectoryBuilder(trajectoryStrafe.end())
                .splineTo(new Vector2d(0.0, 0.0), Math.toRadians(0.0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Follow the trajectories
        drive.followTrajectory(trajectoryForward);
        drive.followTrajectory(trajectoryStrafe);
        drive.followTrajectory(trajectorySpline);
    }
}