package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "MainAuto", group = "")

public class MainAuto extends LinearOpMode {
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
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
    boolean bool = true;
    double powerconstant;
    double shotconstant;
    double turnconstant;
    double driveconstant;
    double basevoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    double drivepower = 0.6;
    double targetshooterpower = 0.34; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    double targetshotpower = 0.42; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    double turnpower = 0.5; //EDIT THIS TO CHANGE THE POWER OF THE DRIVETRAIN FOR THE POWER SHOT
    int armposition;
    int initialHeading, leftHeading, rightHeading;
    int heading;
    int difference;
    boolean position = false;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AW7vQeL/////AAABmZpV+TNdzEfigTTiCS83WyFkYs/PO6Vt1jU0nmyH+MkdBjFiWRCtrz2eL6Dx0LsXgcfEn4iF52EM1s86ALJgZOFpycoesjV/VDzvwjHY+b0gPTTtBwaioglg3HY1rwHz3po8fmRqtDmVqhNG+jYfmwVzi2Suygk8RM0f27sbt1rpZhl08Q+PR+sDV5LirITAa3CKsyISroBs39r+Z1M1XLOvtG0BUKxZWq9ht7z0dCR1bJ1Y2+HaaOodCxz7DZU644E+KlM0PsYidbqb/mhN+Ec17a39TPACFEVrKCYFNsLPFAkcyceJegGpYb3lHp8h/kVoBZ2cZVWq5MDlUpym/QqxhlpcJ4kWuHmGPR4TTpv/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
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
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
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
        Hopper.setPosition(0.92);
        close();
        Cam.setPosition(1);
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
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(1.5, 1.78);
        }
        wait(500);
        initialHeading = (int) getAngle();
        if(initialHeading <=270 && initialHeading >=90){
            leftHeading = initialHeading + 90;
            rightHeading = initialHeading - 90;  }
        else if (initialHeading >270){
            leftHeading = 90 - (360 - initialHeading);
            rightHeading = initialHeading - 90;
        } else if(initialHeading <90){
            leftHeading = initialHeading + 90;
            rightHeading = 360 - (90 - initialHeading);
        }
        telemetry.addData("Initial heading: ", initialHeading);
        wait(500);
        /** Wait for the game to begin */
        telemetry.addLine();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (tfod != null) {
                    wait(500);
                    MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0 ) {
                            // empty list.  no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            a();
                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                // check label to see which target zone to go after.
                                if (recognition.getLabel().equals("Single")) {
                                    telemetry.addData("Target Zone", "B");
                                    b();
                                } else if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Target Zone", "C");
                                    c();
                                } else {
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                }
                            }
                        }

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void a(){
        Cam.setPosition(0.2);
        GoToShoot();
        GoToA();
        wait(30000);
    }
    public void b(){
        Cam.setPosition(0.2);
        GoToShoot();
        //ShootB();
        GoToB();
        wait(30000);

    }
    public void c(){
        Cam.setPosition(0.2);
        GoToShoot();
        //ShootC();
        GoToC();
        GoToLine();
        wait(30000);
    }
    public void TurnLeft(){ //90 degree turn left
        TurnLeftEncoders(0.4,360);
    }
    public void TurnRight(){ //90 degree turn right
        TurnRightEncoders(0.4,360);
    }
    public void GoLeft(){ //45 degree turn left
        TurnLeftEncoders(0.4,190);
    }
    public void GoRight(){ //45 degree turn right
        TurnRightEncoders(0.4,190);
    }
    public void GoToShoot(){
        DriveForwardEncoders(0.6, 2550);
        wait(300);
        TurnLeft();
        correctLeft();
        wait(200);
        DriveForwardEncoders(0.4, 920);
        wait(300);
        TurnRight();
        wait(300);
        correct180();
        ShootGoal();

    }
    public void ShootB(){ //collect 1 ring from the starter stack and shoot it
        ArmEncoder(1,-400);
        ArmEncoder(1,400);
        MIntake.setPower(1);
        MRoller.setPower(1);
        DriveForwardEncoders(0.4,-600);
        DriveForwardEncoders(0.4,530);
        wait(2000);
        MIntake.setPower(0);
        MRoller.setPower(0);
        ShootOne();

    }
    public void ShootC(){ //collect 1 ring from the starter stack and shoot it
        ArmEncoder(1,-400);
        ArmEncoder(1,400);
        MIntake.setPower(1);
        MRoller.setPower(1);
        DriveForwardEncoders(0.4,-1250);
        DriveForwardEncoders(0.4,1140);
        wait(3000);
        MIntake.setPower(0);
        MRoller.setPower(0);
        ShootGoal();

    }
    public void GoToA(){
        TurnRightEncoders(0.4,250);
        ArmEncoder(1,-3200);
        MArm.setPower(0);
        DriveForwardEncoders(0.6,270);
        open();
        wait(500);
        DriveForwardEncoders(0.6,-290);
        TurnLeftEncoders(0.4,250);
    }
    public void GoToB(){
        ArmEncoder(1,-3200);
        MArm.setPower(0);
        DriveForwardEncoders(0.6,800);
        open();
        wait(500);
        DriveForwardEncoders(0.6,-600);


    }
    public void GoToC(){
        DriveForwardEncoders(0.6,1500);
        GoRight();
        ArmEncoder(1,-4000);
        MArm.setPower(0);
        DriveForwardEncoders(0.6,700);
        open();
        wait(500);
        DriveForwardEncoders(0.6,-700);
        GoLeft();

    }
    public void GoToLine(){
        DriveForwardEncoders(0.6,-900);
    }
    public void GetGoalA(){
        open();
        TurnLeftEncoders(0.4,710);
        wait(200);
        DriveForwardEncoders(0.6,2000);
        close();
        ClawUp();

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

        ArmEncoder(1,3200);
        MArm.setPower(0);

    }
    public void ClawDown(){
        ArmEncoder(1,-3200);
        MArm.setPower(0);

    }
    public void ShootGoal(){
        //shooting code



        Hopper.setPosition(0.94); //set arm back

        MLeftShooter.setPower(1);
        MRightShooter.setPower(-1);
        wait(1500);
        for(int i=0; i<3; i++){ //shoot the rings
            Hopper.setPosition(0.83);
            wait(250);
            Hopper.setPosition(0.94);
            wait(250);
        }
        Hopper.setPosition(0.94); // bring the arm back
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);

    }
    public void ShootOne(){ //shoot one ring
        //shooting code



        Hopper.setPosition(0.94); //set arm back

        MLeftShooter.setPower(1);
        MRightShooter.setPower(-1);
        wait(1000);
        //shoot the ring
        Hopper.setPosition(0.83);
        wait(200);
        Hopper.setPosition(0.94); // bring the arm back
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);

    }
    public void PowerShot(){
        //shooting code
        voltage = ExpansionHub1_VoltageSensor.getVoltage();
        MIntake.setPower(-0.5);
        wait(800);
        MIntake.setPower(0);
        MLeftShooter.setPower(shotconstant/voltage);
        MRightShooter.setPower(-shotconstant/voltage);
        wait(1000);
        MIntake.setPower(1); //center
        wait(500);
        MIntake.setPower(0);

        voltage = ExpansionHub1_VoltageSensor.getVoltage();
        MLeft.setPower(turnconstant/voltage);
        MRight.setPower(turnconstant/voltage);
        wait(180);
        MLeft.setPower(0);
        MRight.setPower(0);
        //left
        MIntake.setPower(1);
        wait(500);
        MIntake.setPower(0);

        voltage = ExpansionHub1_VoltageSensor.getVoltage();
        MLeft.setPower(-turnconstant/voltage);
        MRight.setPower(-turnconstant/voltage);
        wait(180);
        MLeft.setPower(0);
        MRight.setPower(0);
        //right
        MIntake.setPower(1);
        wait(500);
        MIntake.setPower(0);
        MLeftShooter.setPower(0);
        MRightShooter.setPower(0);
    }




    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void DriveForwardEncoders(double power, int distance)
    {
        MLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MLeft.setDirection(DcMotor.Direction.REVERSE);
        MRight.setDirection(DcMotor.Direction.FORWARD);
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
    public void TurnRightEncoders(double power, int distance)
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
        MRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position using encoders and stop with brakes on
        MIntake.setTargetPosition(distance);
        MRoller.setTargetPosition(distance);
        MIntake.setPower(power);
        MRoller.setPower(power);
        MIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && MIntake.isBusy() && MRoller.isBusy())
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
        while (opModeIsActive() && MArm.isBusy())
        {
            telemetry.addData("encoder-Arm", MArm.getCurrentPosition() + " busy=" + MArm.isBusy());

            telemetry.update();
            idle();
        }
        MArm.setPower(0);
        MArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder count of the right motor
        MArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sets right motor ro run to a target position usiong encoders and stop with brakes on

    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double angle = (double)angles.firstAngle;
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
                MLeft.setPower(0.15);
                MRight.setPower(0.15);
            } else if(getAngle() > initialHeading) {
                MLeft.setPower(-0.15);
                MRight.setPower(-0.15);
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
    public void correctLeft(){
        MLeft.setDirection(DcMotor.Direction.FORWARD);
        MRight.setDirection(DcMotor.Direction.FORWARD);
        while(getAngle()!= leftHeading){
            if(getAngle()< leftHeading){
                MLeft.setPower(0.15);
                MRight.setPower(0.15);
            } else if(getAngle() > leftHeading) {
                MLeft.setPower(-0.15);
                MRight.setPower(-0.15);
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
    public void correctRight(){
        MLeft.setDirection(DcMotor.Direction.FORWARD);
        MRight.setDirection(DcMotor.Direction.FORWARD);
        while(getAngle()!= rightHeading){
            if(getAngle()< rightHeading){
                MLeft.setPower(0.15);
                MRight.setPower(0.15);
            } else if(getAngle() > rightHeading) {
                MLeft.setPower(-0.15);
                MRight.setPower(-0.15);
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

}

