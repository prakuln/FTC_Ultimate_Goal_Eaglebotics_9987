package org.firstinspires.ftc.teamcode;

//THIS IS A BETA VERSION FOR THE TELEOP CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "TeleOp_Beta", group = "")
public class TeleOp_Beta extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Orientation lastAngles = new Orientation();
    int initialHeading = 186;
    private DcMotor MRight;
    private DcMotor MLeft;
    private DcMotor MRightShooter;
    private DcMotor MLeftShooter;
    private DcMotor MIntake;
    private DcMotor MRoller;
    private DcMotor MArm;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Cam;
    private Servo Hopper;
    private VoltageSensor ExpansionHub1_VoltageSensor;
    double ldistance;
    double rdistance;
    float RstickX;
    float RstickY;
    double LStick;
    double clawangle;
    double voltage;
    double powerconstant;
    double shotconstant;
    double turnconstant;
    double driveconstant;
    double basevoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    double drivepower = 0.6;
    double targetshooterpower = 0.45; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    double targetshotpower = 0.4; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    double turnpower = 0.5; //EDIT THIS TO CHANGE THE POWER OF THE DRIVETRAIN FOR THE POWER SHOT
    int armposition;
    double speed = 1;
    double contPower;
    double LT;
    double RT;
    boolean speedmode = false;
    boolean position = true;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

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
    @Override
    public void runOpMode() {

        MRight = hardwareMap.dcMotor.get("MRight"); //initialize the motors
        MLeft = hardwareMap.dcMotor.get("MLeft");
        MLeftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        MRightShooter = hardwareMap.dcMotor.get("MRightShooter");
        MIntake = hardwareMap.dcMotor.get("MIntake");
        MRoller = hardwareMap.dcMotor.get("MRoller");
        MArm = hardwareMap.dcMotor.get("MArm");
        ExpansionHub1_VoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        LClaw = hardwareMap.servo.get("LClaw");
        RClaw = hardwareMap.servo.get("RClaw");
        Cam = hardwareMap.servo.get("Cam");
        Hopper = hardwareMap.servo.get("Hopper");
        Cam.setPosition(0.2);
        Hopper.setPosition(0.94);
        open();
        BNO055IMU.Parameters imuParameters;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
                Hopper.setPosition(0.94);
                voltage = ExpansionHub1_VoltageSensor.getVoltage();
                powerconstant = basevoltage*targetshooterpower;
                shotconstant = basevoltage*targetshotpower;
                turnconstant = basevoltage*turnpower;
                driveconstant = basevoltage*drivepower;
                RstickX = gamepad1.right_stick_x; //the variable will collect the value from the controller
                RstickY = gamepad1.right_stick_y;
                LStick= gamepad1.left_stick_y;
                RT = gamepad1.right_trigger;
                LT = gamepad1.left_trigger;
                RstickX*=0.7; //speed for turns
                MRight.setPower(-(-RstickX-RstickY)*speed*0.92); // motor speed can be adjusted
                MLeft.setPower(-(-RstickX+RstickY)*speed);
                if (gamepad1.y) {
                    voltage = ExpansionHub1_VoltageSensor.getVoltage();
                    MLeftShooter.setPower(-powerconstant/voltage);
                    MRightShooter.setPower(powerconstant/voltage);
                }
                if (gamepad1.a) {
                    MLeftShooter.setPower(powerconstant/voltage);
                    MRightShooter.setPower(-powerconstant/voltage);
                    wait(500);
                    MLeftShooter.setPower(0);
                    MRightShooter.setPower(0);
                }
                if (gamepad1.left_bumper){ //code for speed change
                    if(speedmode == false) {
                        SpeedControl(0.6);
                        speedmode = true;
                    }

                    else {
                        SpeedControl(1);
                        speedmode = false;
                    }


                }
                if (gamepad1.x){
                    ShootGoal();

                }
                if (gamepad1.b){
                    ShootOne();

                }
                if (gamepad1.right_bumper){ //code for shooting the power shots
                    PowerShot();
                }


                if (gamepad1.dpad_down){ // code for the arm
                    MArm.setPower(-1);
                }
                else if (gamepad1.dpad_up){
                    MArm.setPower(1);
                }
                else{
                    MArm.setPower(0);
                }
                if (gamepad1.dpad_right){
                    close();
                }
                if (gamepad1.dpad_left){
                    open();
                }

                MIntake.setPower(-LStick);
                MRoller.setPower(-LStick);

            }

        }

    }
    public void open(){
        LClaw.setPosition(0.6);
        RClaw.setPosition(1);
    }
    public void close(){
        LClaw.setPosition(0);
        RClaw.setPosition(0);
    }
    public void ClawUp(){

        wait(500);
        ArmEncoder(1,-3200);
        MArm.setPower(0);
        position = false;

    }
    public void ClawDown(){

        ArmEncoder(1,3200);
        MArm.setPower(0);
        MArm.setPower(0);
        position = true;

    }
    public void SpeedControl(double speedcontr){
        speed = speedcontr;
    }
    public void ShootGoal(){
        //shooting code



        Hopper.setPosition(0.945); //set arm back

        MLeftShooter.setPower(-powerconstant/voltage);
        MRightShooter.setPower(powerconstant/voltage);
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
    public void PowerShot(){
        //shooting code
        voltage = ExpansionHub1_VoltageSensor.getVoltage();
        Hopper.setPosition(0.94);
        MLeftShooter.setPower(-shotconstant/voltage);
        MRightShooter.setPower(shotconstant/voltage);
        wait(1500);
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94);
        turnGyro(4);
        MLeft.setPower(0);
        MRight.setPower(0);
        wait(1000);
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94);
        turnGyro(-6);
        MLeft.setPower(0);
        MRight.setPower(0);
        wait(1000);
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94);
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);
    }
    public void DriveForwardEncoders(double power, int distance)
    {
        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MLeft.setTargetPosition(-distance);
        MRight.setTargetPosition(distance);
        MLeft.setPower(power);
        MRight.setPower(power);
        MLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && MLeft.isBusy() && MRight.isBusy())
        {
            telemetry.addData("encoder-Left", MLeft.getCurrentPosition() + " busy=" + MLeft.isBusy());
            telemetry.addData("encoder-Right", MRight.getCurrentPosition() + " busy=" + MRight.isBusy());

            telemetry.update();
            idle();
        }
        MLeft.setPower(0);
        MRight.setPower(0);
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of left motor
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position usiong encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public void TurnEncoders(double power, int distance) //LEFT IS POSITIVE
    {
        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MLeft.setTargetPosition(distance);
        MRight.setTargetPosition(distance);
        MLeft.setPower(power);
        MRight.setPower(power);
        MLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && MLeft.isBusy() && MRight.isBusy())
        {
            telemetry.addData("encoder-Left", MLeft.getCurrentPosition() + " busy=" + MLeft.isBusy());
            telemetry.addData("encoder-Right", MRight.getCurrentPosition() + " busy=" + MRight.isBusy());

            telemetry.update();
            idle();
        }
        MLeft.setPower(0);
        MRight.setPower(0);
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of left motor
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position usiong encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public void TurnRightEncoders(double power, int distance)
    {
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of the right motor
        MLeft.setDirection(DcMotor.Direction.FORWARD);
        MRight.setDirection(DcMotor.Direction.FORWARD);
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MLeft.setTargetPosition(distance);
        MRight.setTargetPosition(distance);
        MLeft.setPower(power);
        MRight.setPower(power*0.978);
        MLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && ( MLeft.isBusy() || MRight.isBusy()))
        {
            telemetry.addData("encoder-Left", MLeft.getCurrentPosition() + " busy=" + MLeft.isBusy());
            telemetry.addData("encoder-Right", MRight.getCurrentPosition() + " busy=" + MRight.isBusy());

            telemetry.update();
            idle();
        }
        MLeft.setPower(0);
        MRight.setPower(0);
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of left motor
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position usiong encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public void TurnLeftEncoders(double power, int distance)
    {
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MLeft.setDirection(DcMotor.Direction.REVERSE);
        MRight.setDirection(DcMotor.Direction.REVERSE);
        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MLeft.setTargetPosition(distance);
        MRight.setTargetPosition(distance);
        MLeft.setPower(power);
        MRight.setPower(power*0.978);
        MLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sets left motor ro run to a target position using encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && ( MLeft.isBusy() || MRight.isBusy()))
        {
            telemetry.addData("encoder-Left", MLeft.getCurrentPosition() + " busy=" + MLeft.isBusy());
            telemetry.addData("encoder-Right", MRight.getCurrentPosition() + " busy=" + MRight.isBusy());

            telemetry.update();
            idle();
        }
        MLeft.setPower(0);
        MRight.setPower(0);
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of left motor
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //resets encoder count of the right motor
        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets left motor ro run to a target position usiong encoders and stop with brakes on
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

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
        while (opModeIsActive() && MIntake.isBusy())
        {
            telemetry.addData("encoder-Intake", MIntake.getCurrentPosition() + " busy=" + MIntake.isBusy());

            telemetry.update();
            idle();
        }
        MIntake.setPower(0);
        MIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of the right motor
        MIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public void ArmEncoder(double power, int distance)
    {
        //resets encoder count of the right motor
        MArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MArm.setTargetPosition(distance);
        MArm.setPower(power);
        MArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MArm.setPower(0);
        MArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of the right motor
        MArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }
    public void ShootOne(){ //shoot one ring
        //shooting code



        Hopper.setPosition(0.94); //set arm back

        MLeftShooter.setPower(-powerconstant/voltage);
        MRightShooter.setPower(powerconstant/voltage);
        wait(1000);
        //shoot the ring
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94); // bring the arm back
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);

    }  private int getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        int angle = (int)angles.firstAngle;
        if(angle <0){
            angle += 360;
        }
        return angle;
    }
    public void correct180(){
        MLeft.setDirection(DcMotor.Direction.FORWARD);
        MRight.setDirection(DcMotor.Direction.FORWARD);
        while(getAngle()!= initialHeading){
            if(getAngle()< initialHeading){
                MLeft.setPower(0.3);
                MRight.setPower(0.3);
            } else if(getAngle() > initialHeading) {
                MLeft.setPower(-0.3);
                MRight.setPower(-0.3);
            } else {
                MLeft.setPower(0);
                MRight.setPower(0);
                break;}
            telemetry.addData("Heading ", getAngle());
            telemetry.update();
        }
        MLeft.setPower(0);
        MRight.setPower(0);
        wait(100);
    }
    public void turnGyro(int angle) { // positive is LEFT, negative is RIGHT
        MLeft.setDirection(DcMotor.Direction.FORWARD);
        MRight.setDirection(DcMotor.Direction.FORWARD);
        int currentAngle = (int) getAngle();
        int targetAngle = currentAngle + angle;
        while(getAngle()!= targetAngle){
            telemetry.addData("current heading:", currentAngle);
            telemetry.update();
            if(currentAngle < targetAngle){
                MLeft.setPower(-0.5);
                MRight.setPower(-0.5);
            } else if(currentAngle > targetAngle) {
                MLeft.setPower(0.5);
                MRight.setPower(0.5);
            } else {
                MLeft.setPower(0);
                MRight.setPower(0);
                break;}
        }
    }
}