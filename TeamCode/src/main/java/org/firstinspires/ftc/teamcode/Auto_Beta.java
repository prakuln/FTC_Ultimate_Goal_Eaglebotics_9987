package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
//THIS IS A BETA VERSION FOR THE AUTONOMOUS CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

@Autonomous(name = "Auto_Beta", group = "")

public class Auto_Beta extends LinearOpMode {
    // The IMU sensor object
    BNO055IMU imu;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VoltageSensor ExpansionHub1_VoltageSensor;
    private static final String VUFORIA_KEY =
            "AW7vQeL/////AAABmZpV+TNdzEfigTTiCS83WyFkYs/PO6Vt1jU0nmyH+MkdBjFiWRCtrz2eL6Dx0LsXgcfEn4iF52EM1s86ALJgZOFpycoesjV/VDzvwjHY+b0gPTTtBwaioglg3HY1rwHz3po8fmRqtDmVqhNG+jYfmwVzi2Suygk8RM0f27sbt1rpZhl08Q+PR+sDV5LirITAa3CKsyISroBs39r+Z1M1XLOvtG0BUKxZWq9ht7z0dCR1bJ1Y2+HaaOodCxz7DZU644E+KlM0PsYidbqb/mhN+Ec17a39TPACFEVrKCYFNsLPFAkcyceJegGpYb3lHp8h/kVoBZ2cZVWq5MDlUpym/QqxhlpcJ4kWuHmGPR4TTpv/";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        Robot.MLeftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.MRightShooter = hardwareMap.dcMotor.get("MRightShooter");
        Robot.MIntake = hardwareMap.dcMotor.get("MIntake");
        Robot.MArm = hardwareMap.dcMotor.get("MArm");
        ExpansionHub1_VoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.LClaw = hardwareMap.servo.get("LClaw");
        Robot.RClaw = hardwareMap.servo.get("RClaw");
        Robot.Cam = hardwareMap.servo.get("Cam");
        Robot.Hopper = hardwareMap.servo.get("Hopper");
        Robot.Hopper.setPosition(0.92);
        Robot.closeArm();
        Robot.Cam.setPosition(0.9);
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

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 1.78);
        }
        Robot.wait(500);
        Robot.wait(500);
        telemetry.addLine();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (tfod != null) {
                    Robot.wait(500);
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
        Robot.Cam.setPosition(0.2);
        Robot.DriveForwardEncoders(0.6,2800);
        Robot.ArmDown();
        Robot.openArm();
        //GoToShoot();
        //GoToA();
        Robot.wait(30000);
    }

    public void b(){
        Robot.Cam.setPosition(0.2);
        Robot.DriveForwardEncoders(0.6,2800);
        Robot.ArmDown();
        Robot.openArm();
        //GoToShoot();
        //ShootB();
        //GoToB();
        Robot.wait(30000);

    }

    public void c(){
        Robot.Cam.setPosition(0.2);
        Robot.DriveForwardEncoders(0.6,2800);
        Robot.ArmDown();
        Robot.openArm();
        //GoToShoot();
        //ShootC();
        //GoToC();
        //GoToLine();
        Robot.wait(30000);
    }

    public void TurnLeft(){ //90 degree turn left
        Robot.TurnLeftEncoders(0.4,370);
    }

    public void TurnRight(){ //90 degree turn right
        Robot.TurnRightEncoders(0.4,360);
    }

    public void GoLeft(){ //45 degree turn left
        Robot.TurnLeftEncoders(0.4,190);
    }

    public void GoRight(){ //45 degree turn right
        Robot.TurnRightEncoders(0.4,190);
    }

    public void GoToShoot(){
        Robot.DriveForwardEncoders(0.6, 2650);
        Robot.wait(300);
        TurnLeft();
        //correctLeft();
        Robot.wait(200);
        Robot.DriveForwardEncoders(0.4, 920);
        Robot.wait(300);
        TurnRight();
        Robot.wait(300);
        //correct180();
        Robot.ShootGoal(ExpansionHub1_VoltageSensor.getVoltage());

    }

    public void ShootB(){ //collect 1 ring from the starter stack and shoot it
        Robot.ArmEncoder(1,-400);
        Robot.ArmEncoder(1,400);
        Robot.MIntake.setPower(1);
        Robot.DriveForwardEncoders(0.4,-600);
        Robot.DriveForwardEncoders(0.4,530);
        Robot.wait(2000);
        Robot.MIntake.setPower(0);
        Robot.ShootOne(ExpansionHub1_VoltageSensor.getVoltage());

    }

    public void ShootC(){ //collect 1 ring from the starter stack and shoot it
        Robot.ArmEncoder(1,-400);
        Robot.ArmEncoder(1,400);
        Robot.MIntake.setPower(1);
        Robot.DriveForwardEncoders(0.4,-1250);
        Robot.DriveForwardEncoders(0.4,1140);
        Robot.wait(3000);
        Robot.MIntake.setPower(0);
        Robot.ShootGoal(ExpansionHub1_VoltageSensor.getVoltage());

    }

    public void GoToA(){
        Robot.TurnRightEncoders(0.4,250);
        Robot.ArmEncoder(1,-3200);
        Robot.MArm.setPower(0);
        Robot.DriveForwardEncoders(0.6,270);
        Robot.openArm();
        Robot.wait(500);
        Robot.DriveForwardEncoders(0.6,-290);
        Robot.TurnLeftEncoders(0.4,250);
    }

    public void GoToB(){
        Robot.ArmEncoder(1,-3200);
        Robot.MArm.setPower(0);
        Robot.DriveForwardEncoders(0.6,800);
        Robot.openArm();
        Robot.wait(500);
        Robot.DriveForwardEncoders(0.6,-600);


    }

    public void GoToC(){
        Robot.DriveForwardEncoders(0.6,1500);
        GoRight();
        Robot.ArmEncoder(1,-4000);
        Robot.MArm.setPower(0);
        Robot.DriveForwardEncoders(0.6,700);
        Robot.openArm();
        Robot.wait(500);
        Robot.DriveForwardEncoders(0.6,-700);
        GoLeft();

    }

    public void GoToLine(){
        Robot.DriveForwardEncoders(0.6,-900);
    }

    public void GetGoalA(){
        Robot.openArm();
        Robot.TurnLeftEncoders(0.4,710);
        Robot.wait(200);
        Robot.DriveForwardEncoders(0.6,2000);
        Robot.closeArm();
        Robot.ArmUp();

    }

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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}


