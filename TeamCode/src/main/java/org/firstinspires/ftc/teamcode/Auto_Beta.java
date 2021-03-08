package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.mech_drive.MyMecanumDrive;
import org.firstinspires.ftc.teamcode.mech_drive.StandardTrackingWheelLocalizer;

import java.util.List;
//THIS IS A BETA VERSION FOR THE AUTONOMOUS CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

@Autonomous(name = "Auto_Beta", group = "OpModes")

public class Auto_Beta extends LinearOpMode {
    @Override
   public void runOpMode(){
        initVuforia();
        initTfod();
        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        Robot.leftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.rightShooter = hardwareMap.dcMotor.get("MRightShooter");
        Robot.mIntake = hardwareMap.dcMotor.get("MIntake");
        Robot.mArm = hardwareMap.dcMotor.get("MArm");
        Robot.voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.leftClaw = hardwareMap.servo.get("LClaw");
        Robot.rightClaw = hardwareMap.servo.get("RClaw");
        Robot.cam = hardwareMap.servo.get("Cam");
        Robot.hopper = hardwareMap.servo.get("Hopper");
        Robot.hopperBack();
        Robot.closeArm();
        Robot.cameraOut();
        Robot.drive = new MyMecanumDrive(hardwareMap);
        Robot.myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        Robot.myLocalizer.setPoseEstimate(Coordinates.start);
        Robot.setCameraZoom();
        telemetry.addLine();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (Robot.tfod != null) {
                    Robot.wait(500);
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = Robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0 ) {
                            // empty list.  no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            telemetry.update();
                            Robot.a();
                            Robot.wait(30000);
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
                                    telemetry.update();
                                    Robot.b();
                                    Robot.wait(30000);
                                } else if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Target Zone", "C");
                                    telemetry.update();
                                    Robot.c();
                                    Robot.wait(30000);
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

        if (Robot.tfod != null) {
            Robot.tfod.shutdown();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Robot.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        Robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        Robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, Robot.vuforia);
        Robot.tfod.loadModelFromAsset(Robot.TFOD_MODEL_ASSET, Robot.LABEL_FIRST_ELEMENT, Robot.LABEL_SECOND_ELEMENT);
    }


}


