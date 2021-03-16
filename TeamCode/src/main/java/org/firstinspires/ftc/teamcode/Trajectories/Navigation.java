package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.Mecanum_Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Shooter;

import java.util.Arrays;

public class Navigation {
    public static void goToShoot(){
        Robot.drive.setPoseEstimate(Coordinates.start);
        Trajectory trajectory = Robot.drive.trajectoryBuilder(Coordinates.start)
                .splineTo(new Vector2d(Coordinates.auto_point.getX(), Coordinates.auto_point.getY()), Coordinates.auto_point.getHeading())
                .splineTo(new Vector2d(Coordinates.shoot.getX(), Coordinates.shoot.getY()), Coordinates.shoot.getHeading() , new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(4),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1, () -> Arm.moveDown(1))
                .addTemporalMarker(1.6, Arm::stop)
                .build();
        Shooter.on(Constants.powerConstant);
        Robot.drive.followTrajectory(trajectory);
        Shooter.shootThree();
        Shooter.off();
    }
    public static void shootStack(int state){//1 is B, 4 is C (in A there is no stack)
        //drive.setPoseEstimate(Coordinates.shoot);
        Intake.succIn(1);
        switch (state) {
            case 1:
                Trajectory trajectory = Robot.drive.trajectoryBuilder(Coordinates.shoot)
                        .back(15, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                Trajectory trajectory1 = Robot.drive.trajectoryBuilder(trajectory.end())
                        .forward(15, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                Shooter.on(Constants.powerConstant);
                Robot.drive.followTrajectory(trajectory);
                Robot.wait(500);
                Robot.drive.followTrajectory(trajectory1);
                Robot.wait(500);
                Shooter.shootOne();
                Intake.stop();
                Shooter.off();
                break;
            case 4:
                Trajectory trajectory2 = Robot.drive.trajectoryBuilder(Coordinates.shoot)
                        .back(15, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                Trajectory trajectory3 = Robot.drive.trajectoryBuilder(trajectory2.end())
                        .forward(15, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                Shooter.on(Constants.powerConstant);
                Robot.drive.followTrajectory(trajectory2);
                Robot.wait(500);
                Robot.drive.followTrajectory(trajectory3);
                Robot.wait(500);
                Shooter.shootThree();
                Intake.stop();
                Shooter.off();
                break;
        }
    }
    public static void goToZone(int state){
        //drive.setPoseEstimate(Coordinates.shoot);
        switch (state) {
            case 0: //A
                Trajectory trajectory = Robot.drive.trajectoryBuilder(Coordinates.shoot)
                        .splineTo(new Vector2d(Coordinates.a.getX(), Coordinates.a.getY()), Coordinates.a.getHeading(), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addTemporalMarker(0, () -> Arm.moveDown(1))
                        .addTemporalMarker(1.5, Arm::stop)
                        .build();
                Robot.drive.followTrajectory(trajectory);
                break;
            case 1://B
                Trajectory trajectory1 = Robot.drive.trajectoryBuilder(Coordinates.shoot)
                        .splineTo(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY()), Coordinates.b.getHeading())
                        .addTemporalMarker(0, () -> Arm.moveDown(1))
                        .addTemporalMarker(1.5, Arm::stop)
                        .build();
                Robot.drive.followTrajectory(trajectory1);
                break;
            case 4://C
                Trajectory trajectory2 = Robot.drive.trajectoryBuilder(Coordinates.shoot)
                        .splineTo(new Vector2d(Coordinates.c.getX(), Coordinates.c.getY()), Coordinates.c.getHeading())
                        .addTemporalMarker(0, () -> Arm.moveDown(1))
                        .addTemporalMarker(1.5, Arm::stop)
                        .build();
                Robot.drive.followTrajectory(trajectory2);
                break;
            }
            Robot.wait(500);
            Arm.open();


    }
    public static void getWobble(int state){ //0 is A, 1 is B, 4 is C
        switch (state){
            case 0: //A
            Trajectory trajectory1 = Robot.drive.trajectoryBuilder(Coordinates.a)
                    .back(20)
                    .build();
            Robot.drive.followTrajectory(trajectory1);
            Robot.drive.turn(Math.toRadians(-90));
            Trajectory trajectory = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            Robot.drive.followTrajectory(trajectory);
            break;
            case 1://B
            Trajectory trajectory2 = Robot.drive.trajectoryBuilder(Coordinates.b)
                    .back(20)
                    .build();
            Robot.drive.followTrajectory(trajectory2);
            Robot.drive.turn(Math.toRadians(180));
            Trajectory trajectory3 = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            Robot.drive.followTrajectory(trajectory3);
            break;
            case 4://C
            Trajectory trajectory4 = Robot.drive.trajectoryBuilder(Coordinates.c)
                    .back(20)
                    .build();
            Robot.drive.followTrajectory(trajectory4);
            Robot.drive.turn(Math.toRadians(180));
            Trajectory trajectory5 = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            Robot.drive.followTrajectory(trajectory5);
            break;
        }
        Arm.close();
        Robot.wait(500);
    }
    public static void bringWobble(int state){//0 is A, 1 is B, 4 is C
        Robot.drive.turn(Math.toRadians(180));
        switch (state){
            case 0: //A
            Trajectory trajectory = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.a.getX()-2, Coordinates.a.getY()+7), Coordinates.a.getHeading())
                    .build();
            Robot.drive.followTrajectory(trajectory);
            break;
            case 1://B
            Trajectory trajectory1 = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY()), 358)
                    .build();
            Robot.drive.followTrajectory(trajectory1);
            break;
            case 4://C
            Trajectory trajectory2 = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.c.getX() -2, Coordinates.c.getY() - 3), Coordinates.c.getHeading())
                    .build();
            Robot.drive.followTrajectory(trajectory2);
            break;
        }
        Arm.open();
        Robot.wait(500);
    }
    public static void goToLine(int state){
        switch (state){
            case 0: //A
            Trajectory trajectory = Robot.drive.trajectoryBuilder(Coordinates.a)
                    .back(5)
                    .build();
            Robot.drive.followTrajectory(trajectory);
            break;
            case 1://B
            Trajectory trajectory2 = Robot.drive.trajectoryBuilder(Coordinates.b)
                    .back(5)
                    .build();
            Robot.drive.followTrajectory(trajectory2);
            break;
            case 4://C
            Trajectory trajectory1 = Robot.drive.trajectoryBuilder(Coordinates.c)
                    .back(20)
                    .build();
            Robot.drive.followTrajectory(trajectory1);
            Robot.drive.turn(Math.toRadians(45));
            Trajectory trajectory3 = Robot.drive.trajectoryBuilder(Robot.drive.getPoseEstimate())
                    .back(30)
                    .build();
            Robot.drive.followTrajectory(trajectory3);
            break;
        }

    }
}
