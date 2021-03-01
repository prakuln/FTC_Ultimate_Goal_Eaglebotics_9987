package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.mech_drive.DriveConstants;
import org.firstinspires.ftc.teamcode.mech_drive.MyMecanumDrive;

import java.util.Arrays;

public class Robot {
    public static VuforiaLocalizer vuforia;
    public static TFObjectDetector tfod;
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static final String VUFORIA_KEY =
            "AW7vQeL/////AAABmZpV+TNdzEfigTTiCS83WyFkYs/PO6Vt1jU0nmyH+MkdBjFiWRCtrz2eL6Dx0LsXgcfEn4iF52EM1s86ALJgZOFpycoesjV/VDzvwjHY+b0gPTTtBwaioglg3HY1rwHz3po8fmRqtDmVqhNG+jYfmwVzi2Suygk8RM0f27sbt1rpZhl08Q+PR+sDV5LirITAa3CKsyISroBs39r+Z1M1XLOvtG0BUKxZWq9ht7z0dCR1bJ1Y2+HaaOodCxz7DZU644E+KlM0PsYidbqb/mhN+Ec17a39TPACFEVrKCYFNsLPFAkcyceJegGpYb3lHp8h/kVoBZ2cZVWq5MDlUpym/QqxhlpcJ4kWuHmGPR4TTpv/";

    public static DcMotor mArm;
    public static DcMotor leftFront;
    public static DcMotor leftRear;
    public static DcMotor rightFront;
    public static DcMotor rightRear;
    public static DcMotor mIntake;
    public static DcMotor leftShooter;
    public static DcMotor rightShooter;
    public static Servo hopper;
    public static Servo leftClaw;
    public static Servo rightClaw;
    public static Servo cam;
    public static VoltageSensor voltageSensor;
    public static MyMecanumDrive drive;
    public static void wait(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
    public static void MechanumDriveControl(double RX, double RY, double LT, double RT){
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower((-RY + RX + (-LT+RT*0.8))*Constants.speed);
        leftRear.setPower((-RY + RX - (-LT+RT))*Constants.speed);
        rightFront.setPower((-RY - RX - (-LT+RT*0.8))*Constants.speed);
        rightRear.setPower((-RY - RX + (-LT+RT))*Constants.speed);
    }
    public static void shooterOn(double constant){
        leftShooter.setPower(constant/getVoltage());
        rightShooter.setPower(-constant/getVoltage());
    }
    public static void shooterOff(){
        leftShooter.setPower(-1);
        rightShooter.setPower(1);
        wait(500);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public static void ShootOne(){ //shoot one ring
        //shooting code
        hopper.setPosition(0.94); //set arm back
        shooterOn(Constants.powerConstant);
        wait(1000);
        //shoot the ring
        hopper.setPosition(0.83);
        wait(200);
        hopper.setPosition(0.94); // bring the arm back
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public static void ShootGoal(){
        //shooting code
        hopperBack(); //set arm back
        shooterOn(Constants.powerConstant);
        wait(500);
        for(int i=0; i<3; i++){ //shoot the rings
            hopperForward();
            wait(400);
            hopperBack();
            shooterOn(Constants.powerConstant);
            wait(400);
        }
        hopperBack(); // bring the arm back
    }
    public static void PowerShot(){
        //shooting code
        hopperBack();
        shooterOn(Constants.shotConstant);
        wait(1500);
        drive.turn(2);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        shooterOn(Constants.shotConstant);
        wait(500);
        hopperForward();
        wait(200);
        hopperBack();
        drive.turn(-2);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        shooterOn(Constants.shotConstant);
        wait(500);
        hopperForward();
        wait(200);
        hopperBack();
        drive.turn(-2);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        shooterOn(Constants.shotConstant);
        wait(500);
        hopperForward();
        wait(200);
        hopperBack();
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public static void openArm(){
        Robot.leftClaw.setPosition(0.6);
        Robot.rightClaw.setPosition(1);
    }
    public static void closeArm(){
        Robot.leftClaw.setPosition(0);
        Robot.rightClaw.setPosition(0);
    }
    public static void hopperBack(){
        hopper.setPosition(0.92);
    }
    public static void hopperForward(){
        hopper.setPosition(0.81);
    }
    public static void cameraOut(){
        cam.setPosition(0.935);
    }
    public static void cameraIn(){
        cam.setPosition(0.2);
    }
    public static void ArmUp(){
        wait(500);
        Robot.mArm.setPower(-1);
        wait(1500);
        Robot.mArm.setPower(0);
    }
    public static void ArmDown(){
        Robot.mArm.setPower(1);
        wait(1500);
        Robot.mArm.setPower(0);
    }
    public static double getVoltage(){
        return voltageSensor.getVoltage();
    }
    public static void SpeedControl(double speedcontr){
        Constants.speed = speedcontr;
    }
    //METHODS FOR AUTONOMOUS
    public static void a(){
        goToShoot();
        goToZone(0);
        getWobble(0);
        bringWobble(0);
        goToLine(0);
        setEndPose();
    }
    public static void b(){
        goToShoot();
        shootStack(1);
        goToZone(1);
        getWobble(1);
        bringWobble(1);
        goToLine(1);
        setEndPose();
    }
    public static void c(){
        goToShoot();
        shootStack(4);
        goToZone(4);
        getWobble(4);
        bringWobble(4);
        goToLine(4);
        setEndPose();
    }
    public static void goToShoot(){
        drive.setPoseEstimate(Coordinates.start);
        Trajectory trajectory = drive.trajectoryBuilder(Coordinates.start)
                .splineTo(new Vector2d(Coordinates.auto_point.getX(), Coordinates.auto_point.getY()), Coordinates.auto_point.getHeading())
                .splineTo(new Vector2d(Coordinates.shoot.getX(), Coordinates.shoot.getY()), Coordinates.shoot.getHeading())
                .addTemporalMarker(1, () -> {
                    mArm.setPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    mArm.setPower(0);
                })
                .build();
        Robot.shooterOn(Constants.powerConstant);
        drive.followTrajectory(trajectory);
        ShootGoal();
        shooterOff();
    }
    public static void shootStack(int state){//1 is B, 4 is C (in A there is no stack)
        //drive.setPoseEstimate(Coordinates.shoot);
        mIntake.setPower(-1);
        if(state ==1){
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.shoot)
                    .back(15, new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                            )
                    ),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            Trajectory trajectory1 = drive.trajectoryBuilder(trajectory.end())
                    .forward(15, new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                            )
                    ),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            Robot.shooterOn(Constants.powerConstant);
            drive.followTrajectory(trajectory);
            wait(500);
            drive.followTrajectory(trajectory1);
            wait(500);
            ShootOne();
            mIntake.setPower(0);
            shooterOff();
        } else if(state ==4){
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.shoot)
                    .back(20, new MinVelocityConstraint(
                    Arrays.asList(
                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                            new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                    )
            ),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            Trajectory trajectory1 = drive.trajectoryBuilder(trajectory.end())
                    .forward(20, new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                            )
                    ),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            Robot.shooterOn(Constants.powerConstant);
            drive.followTrajectory(trajectory);
            wait(500);
            drive.followTrajectory(trajectory1);
            wait(500);
            ShootGoal();
            mIntake.setPower(0);
            shooterOff();
        }
    }
    public static void goToZone(int state){
        //drive.setPoseEstimate(Coordinates.shoot);
        if (state == 0) { //A
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.shoot)
                    .splineTo(new Vector2d(Coordinates.a.getX(), Coordinates.a.getY()), Coordinates.a.getHeading(), new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                            )
                    ),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(0, () -> {
                        mArm.setPower(1);
                    })
                    .addTemporalMarker(1.5, () -> {
                        mArm.setPower(0);
                    })
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==1){//B
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.shoot)
                    .splineTo(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY()), Coordinates.b.getHeading())
                    .addTemporalMarker(0, () -> {
                        mArm.setPower(1);
                    })
                    .addTemporalMarker(1.5, () -> {
                        mArm.setPower(0);
                    })
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==4){//C
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.shoot)
                    .splineTo(new Vector2d(Coordinates.c.getX(), Coordinates.c.getY()), Coordinates.c.getHeading())
                    .addTemporalMarker(0, () -> {
                        mArm.setPower(1);
                    })
                    .addTemporalMarker(1.5, () -> {
                        mArm.setPower(0);
                    })
                    .build();
            drive.followTrajectory(trajectory);

        }
        wait(500);
        openArm();


    }
    public static void getWobble(int state){ //0 is A, 1 is B, 4 is C
        if (state == 0) { //A
            Trajectory trajectory1 = drive.trajectoryBuilder(Coordinates.a)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(270));
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==1){//B
            Trajectory trajectory1 = drive.trajectoryBuilder(Coordinates.b)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(180));
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==4){//C
            Trajectory trajectory1 = drive.trajectoryBuilder(Coordinates.c)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(180));
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }
        closeArm();
        wait(500);
    }
    public static void bringWobble(int state){//0 is A, 1 is B, 4 is C
        drive.turn(Math.toRadians(180));
        if (state == 0) { //A
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.a.getX()-2, Coordinates.a.getY()-3), Coordinates.a.getHeading())
                    .build();
            drive.followTrajectory(trajectory);
        }else if(state ==1){//B
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY() - 3), 358)
                    .build();
            drive.followTrajectory(trajectory);
        }else if(state ==4){//C
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(Coordinates.c.getX() -2, Coordinates.c.getY() - 3), Coordinates.c.getHeading())
                    .build();
            drive.followTrajectory(trajectory);
        }
        openArm();
        wait(500);
    }
    public static void goToLine(int state){
        if (state == 0) { //A
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.a)
                    .back(5)
                    .build();
            drive.followTrajectory(trajectory);
        }else if(state ==1){//B
            Trajectory trajectory = drive.trajectoryBuilder(Coordinates.b)
                    .back(5)
                    .build();
            drive.followTrajectory(trajectory);
        }else if(state ==4){//C
            Trajectory trajectory1 = drive.trajectoryBuilder(Coordinates.c)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(45));
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(30)
                    .build();
            drive.followTrajectory(trajectory);
        }

    }
    public static void setEndPose(){
        Coordinates.end = drive.getPoseEstimate();
    }
    public static void setCameraZoom(){
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 1.3);
        }
    }

    //TELEOP ALIGNMENT METHODS
    public static void adjustHeading(int headingDegrees){
        double headingRadians = Math.toRadians(headingDegrees);
        Pose2d pose = drive.getPoseEstimate();
        double currentHeading = pose.getHeading();
        double difference = headingRadians-currentHeading;
        drive.turn(difference);
    }
    public static void alignStraight(){
        adjustHeading(0);
    }
    public static void alignLeft(){
        adjustHeading(90);
    }
    public static void alignRight(){
        adjustHeading(270);
    }
    public static void alignToShoot(){
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(Coordinates.shoot.getX(), Coordinates.shoot.getY()), Math.toRadians(0))
                .build();
        Robot.shooterOn(Constants.powerConstant);
        drive.followTrajectory(trajectory);
        alignStraight();
        ShootGoal();
        shooterOff();
    }
    public static void updatePosition(){
        drive.setPoseEstimate(Coordinates.zero);
    }
}

