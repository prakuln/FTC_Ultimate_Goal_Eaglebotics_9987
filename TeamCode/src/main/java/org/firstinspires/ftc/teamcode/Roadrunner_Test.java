package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mech_drive.MyTankDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "")
public class Roadrunner_Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MyTankDrive drive = new MyTankDrive(hardwareMap);
        Pose2d myPose = new Pose2d(-60, -60, Math.toRadians(0));
        drive.setPoseEstimate(myPose);
        Trajectory trajectory1 = drive.trajectoryBuilder(myPose)
                .splineTo(new Vector2d(-25, -60), Math.toRadians(0))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(2.5, -37), Math.toRadians(0))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectory(trajectory1);
        drive.followTrajectory(trajectory2);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
