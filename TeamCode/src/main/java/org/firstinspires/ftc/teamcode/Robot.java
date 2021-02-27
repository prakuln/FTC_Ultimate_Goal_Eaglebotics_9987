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

import org.firstinspires.ftc.teamcode.mech_drive.DriveConstants;
import org.firstinspires.ftc.teamcode.mech_drive.MyMecanumDrive;

import java.util.Arrays;

public class Robot {
    public static DcMotor MArm;
    public static DcMotor leftFront;
    public static DcMotor leftRear;
    public static DcMotor rightFront;
    public static DcMotor rightRear;
    public static DcMotor MIntake;
    public static DcMotor MLeftShooter;
    public static DcMotor MRightShooter;
    public static Servo Hopper;
    public static Servo LClaw;
    public static Servo RClaw;
    public static Servo Cam;
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
    public static void MechanumDriveControl(double RX, double RY, double LX){
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower((-RY + RX + LX*0.8)*Constants.speed);
        leftRear.setPower((-RY + RX - LX)*Constants.speed);
        rightFront.setPower((-RY - RX - LX*0.8)*Constants.speed);
        rightRear.setPower((-RY - RX + LX)*Constants.speed);
    }
    public static void DriveForwardEncoders(double power, int distance)
    {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(distance);
        leftRear.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (( leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy()))
        {

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public static void TurnLeftEncoders(double power, int distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(distance);
        leftRear.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (( leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy()))
        {

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public static void TurnRightEncoders(double power, int distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(distance);
        leftRear.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (( leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy()))
        {

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void IntakeEncoder(double power, int distance)
    {
        //resets encoder count of the right motor
        MIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MIntake.setTargetPosition(distance);
        MIntake.setPower(power);
        MIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ( MIntake.isBusy())
        {

        }
        MIntake.setPower(0);
        MIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of the right motor
        MIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public static void ArmEncoder(double power, int distance)
    {
        //resets encoder count of the right motor
        MArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MArm.setTargetPosition(distance);
        MArm.setPower(power);
        MArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (MArm.isBusy())
        {

        }
        MArm.setPower(0);
        MArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of the right motor
        MArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public static void shooterOn(double voltage, double constant){
        MLeftShooter.setPower(constant/voltage);
        MRightShooter.setPower(-constant/voltage);
    }
    public static void shooterOff(){
        MLeftShooter.setPower(-1);
        MRightShooter.setPower(1);
        wait(500);
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);
    }
    public static void ShootOne(double voltage){ //shoot one ring
        //shooting code



        Hopper.setPosition(0.94); //set arm back

        shooterOn(voltage, Constants.powerconstant);
        wait(1000);
        //shoot the ring
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94); // bring the arm back
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);

    }
    public static void ShootGoal(double voltage){
        //shooting code



        Hopper.setPosition(0.945); //set arm back

        shooterOn(voltage, Constants.powerconstant);
        wait(500);
        for(int i=0; i<3; i++){ //shoot the rings
            Hopper.setPosition(0.83);
            wait(300);
            Hopper.setPosition(0.94);
            wait(300);
        }
        Hopper.setPosition(0.94); // bring the arm back
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);

    }
    public static void PowerShot(double voltage){
        //shooting code
        Hopper.setPosition(0.94);
        shooterOn(voltage, Constants.shotconstant);
        wait(1500);
        TurnLeftEncoders(0.7, Constants.PWR_SHOT_LEFT_TURN);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        wait(500);
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94);
        TurnRightEncoders(0.7, Constants.PWR_SHOT_CENTER_TURN);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        wait(500);
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94);
        TurnRightEncoders(0.7, Constants.PWR_SHOT_RIGHT_TURN);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        wait(500);
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94);
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);
    }
    public static void openArm(){
        Robot.LClaw.setPosition(0.6);
        Robot.RClaw.setPosition(1);
    }
    public static void closeArm(){
        Robot.LClaw.setPosition(0);
        Robot.RClaw.setPosition(0);
    }
    public static void ArmUp(){

        wait(500);
        Robot.MArm.setPower(-1);
        wait(1500);
        Robot.MArm.setPower(0);

    }
    public static void ArmDown(){

        Robot.MArm.setPower(1);
        wait(1500);
        Robot.MArm.setPower(0);

    }
    public static void SpeedControl(double speedcontr){
        Constants.speed = speedcontr;
    }
    //METHODS FOR AUTONOMOUS
    public static void a(double voltage){
        goToShoot(voltage);
        goToZone(0);
        getWobble(0);
        bringWobble(0);
        goToLine(0);
    }
    public static void b(double voltage){
        goToShoot(voltage);
        shootStack(1, voltage);
        MIntake.setPower(0);
        Robot.ShootOne(voltage);
        goToZone(1);
        getWobble(1);
        bringWobble(1);
        goToLine(1);
    }
    public static void c(double voltage){
        goToShoot(voltage);
        shootStack(4, voltage);
        wait(1000);
        Robot.ShootGoal(voltage);
        MIntake.setPower(0);
        goToZone(4);
        getWobble(4);
        bringWobble(4);
        goToLine(4);
    }
    public static void goToShoot(double voltage){
        Pose2d Pose = new Pose2d(Coordinates.start.getX(), Coordinates.start.getY(), Coordinates.start.getHeading());
        drive.setPoseEstimate(Pose);
        Trajectory trajectory = drive.trajectoryBuilder(Pose)
                .splineTo(new Vector2d(Coordinates.auto_point.getX(), Coordinates.auto_point.getY()), Coordinates.auto_point.getHeading())
                .splineTo(new Vector2d(Coordinates.shoot.getX(), Coordinates.shoot.getY()), Coordinates.shoot.getHeading())
                .addTemporalMarker(1, () -> {
                    MArm.setPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    MArm.setPower(0);
                })
                //.splineToConstantHeading(new Vector2d(Coordinates.auto_point.getX(), Coordinates.auto_point.getY()), Coordinates.auto_point.getHeading())
                //.splineToConstantHeading(new Vector2d(Coordinates.shoot.getX(), Coordinates.shoot.getY()), Coordinates.shoot.getHeading())
                .build();
        Robot.shooterOn(voltage, Constants.powerconstant);
        drive.followTrajectory(trajectory);

    }
    public static void shootStack(int state, double voltage){//1 is B, 4 is C (in A there is no stack)
        Pose2d Pose = new Pose2d(Coordinates.shoot.getX(), Coordinates.shoot.getY(), Coordinates.shoot.getHeading());
        drive.setPoseEstimate(Pose);
        MIntake.setPower(-1);
        if(state ==1){

            Trajectory trajectory = drive.trajectoryBuilder(Pose)
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
            Robot.shooterOn(voltage, Constants.powerconstant);
            drive.followTrajectory(trajectory);
            wait(500);
            drive.followTrajectory(trajectory1);
        } else if(state ==4){

            Trajectory trajectory = drive.trajectoryBuilder(Pose)
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
            Robot.shooterOn(voltage, Constants.powerconstant);
            drive.followTrajectory(trajectory);
            wait(500);
            drive.followTrajectory(trajectory1);
        }
    }
    public static void goToZone(int state){

        Pose2d Pose = new Pose2d(Coordinates.shoot.getX(), Coordinates.shoot.getY(), Coordinates.shoot.getHeading());
        drive.setPoseEstimate(Pose);
        if (state == 0) { //A
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.a.getX(), Coordinates.a.getY()), Coordinates.a.getHeading(), new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                            )
                    ),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(0, () -> {
                        MArm.setPower(1);
                    })
                    .addTemporalMarker(1.5, () -> {
                        MArm.setPower(0);
                    })
                    //.splineToLinearHeading(new Vector2d(Coordinates.a.getX(), Coordinates.a.getY()), Coordinates.a.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==1){//B
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY()), Coordinates.b.getHeading())
                    .addTemporalMarker(0, () -> {
                        MArm.setPower(1);
                    })
                    .addTemporalMarker(1.5, () -> {
                        MArm.setPower(0);
                    })
                    //.splineToConstantHeading(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY()), Coordinates.b.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==4){//C
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.c.getX(), Coordinates.c.getY()), Coordinates.c.getHeading())
                    .addTemporalMarker(0, () -> {
                        MArm.setPower(1);
                    })
                    .addTemporalMarker(1.5, () -> {
                        MArm.setPower(0);
                    })
                    //.splineToLinearHeading(new Vector2d(Coordinates.c.getX(), Coordinates.c.getY()), Coordinates.c.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }
        wait(500);
        openArm();


    }
    public static void getWobble(int state){ //0 is A, 1 is B, 4 is C
        if (state == 0) { //A
            Pose2d Pose1 = new Pose2d(Coordinates.a.getX(), Coordinates.a.getY(), Coordinates.a.getHeading());
            drive.setPoseEstimate(Pose1);
            Trajectory trajectory1 = drive.trajectoryBuilder(Pose1)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(270));
            Pose2d Pose = new Pose2d(Coordinates.a.getX(), Coordinates.a.getY()+20, Math.toRadians(180));
            drive.setPoseEstimate(Pose);
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==1){//B
            Pose2d Pose1 = new Pose2d(Coordinates.b.getX(), Coordinates.b.getY(), Coordinates.b.getHeading());
            drive.setPoseEstimate(Pose1);
            Trajectory trajectory1 = drive.trajectoryBuilder(Pose1)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(180));
            Pose2d Pose = new Pose2d(Coordinates.b.getX() - 20, Coordinates.b.getY(), Math.toRadians(180));
            drive.setPoseEstimate(Pose);
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==4){//C
            Pose2d Pose1 = new Pose2d(Coordinates.c.getX(), Coordinates.c.getY(), Coordinates.c.getHeading());
            drive.setPoseEstimate(Pose1);
            Trajectory trajectory1 = drive.trajectoryBuilder(Pose1)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(180));
            Pose2d Pose = drive.getPoseEstimate();
            drive.setPoseEstimate(Pose);
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.wobble.getX(), Coordinates.wobble.getY()), Coordinates.wobble.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }
        closeArm();
        wait(500);
    }
    public static void bringWobble(int state){//0 is A, 1 is B, 4 is C
        drive.turn(Math.toRadians(180));
        Pose2d Pose = new Pose2d(Coordinates.wobble.getX(), Coordinates.wobble.getY(), Math.toRadians(0));
        drive.setPoseEstimate(Pose);
        if (state == 0) { //A
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.a.getX()-2, Coordinates.a.getY()-3), Coordinates.a.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==1){//B
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.b.getX(), Coordinates.b.getY() - 3), 358)
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==4){//C
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .splineTo(new Vector2d(Coordinates.c.getX() -2, Coordinates.c.getY() - 3), Coordinates.c.getHeading())
                    .build();
            drive.followTrajectory(trajectory);

        }
        openArm();
        wait(500);
    }
    public static void goToLine(int state){
        if (state == 0) { //A
            Pose2d Pose = new Pose2d(Coordinates.a.getX(), Coordinates.a.getY(), Coordinates.a.getHeading());
            drive.setPoseEstimate(Pose);
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .back(5)
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==1){//B
            Pose2d Pose = new Pose2d(Coordinates.b.getX(), Coordinates.b.getY() -3, Coordinates.b.getHeading());
            drive.setPoseEstimate(Pose);
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .back(5)
                    .build();
            drive.followTrajectory(trajectory);

        }else if(state ==4){//C
            Pose2d Pose1 = new Pose2d(Coordinates.c.getX(), Coordinates.c.getY(), Math.toRadians(315));
            Trajectory trajectory1 = drive.trajectoryBuilder(Pose1)
                    .back(20)
                    .build();
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(45));
            Pose2d Pose = drive.getPoseEstimate();
            drive.setPoseEstimate(Pose);
            Trajectory trajectory = drive.trajectoryBuilder(Pose)
                    .back(30)
                    .build();
            drive.followTrajectory(trajectory);

        }

    }
    public static void setEndPose(){
        Coordinates.end = drive.getPoseEstimate();
    }
}

