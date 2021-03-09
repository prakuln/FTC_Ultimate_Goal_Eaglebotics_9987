package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class Drivetrain {
    public static void stop(){
        Robot.leftFront.setPower(0);
        Robot.leftRear.setPower(0);
        Robot.rightFront.setPower(0);
        Robot.rightRear.setPower(0);
    }
    public static void speedControl(double speedcontr){
        Constants.speed = speedcontr;
    }
    /*
    public static void adjustHeading(int headingDegrees){
        double headingRadians = Math.toRadians(headingDegrees);
        Pose2d pose = Robot.myLocalizer.getPoseEstimate();
        double currentHeading = pose.getHeading();
        double difference = headingRadians-currentHeading;
        Robot.drive.turn(difference);
    }
    public static void alignStraight(){
        adjustHeading(0);
    }
    public static void alignLeft(){
        adjustHeading(90);
    }
    public static void alignRight(){
        adjustHeading(270);
    }*/
    public static void fieldCentricDrive(double RX, double RY, double LT, double RT){
        Robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -RY*Constants.speed,
                        (LT-RT)*Constants.speed,
                        (-RX)*Constants.speed
                )
        );
    }
    public static void alignToShoot(){
        Trajectory trajectory = Robot.drive.trajectoryBuilder(Robot.myLocalizer.getPoseEstimate())
                .splineTo(new Vector2d(Coordinates.shoot.getX(), Coordinates.shoot.getY() +3), Coordinates.shoot.getHeading())
                .build();
        Shooter.on(Constants.powerConstant);
        Robot.drive.followTrajectory(trajectory);

        //alignStraight();

        //shooterOff();
    }

    public static void setEndPose(){
        Coordinates.end = Robot.drive.getPoseEstimate();
    }
    /*
    public static void updatePosition(){
        Robot.myLocalizer.setPoseEstimate(Coordinates.zero);
    } */
}
