package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mecanum_Drive.MyMecanumDrive;
import org.firstinspires.ftc.teamcode.Mecanum_Drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Trajectories.Coordinates;
import org.firstinspires.ftc.teamcode.Vision.Camera;

public class Robot {
    public static MyMecanumDrive drive;
    public static StandardTrackingWheelLocalizer myLocalizer;
    public static Telemetry tele;
    private static void init(HardwareMap hardwareMap, Telemetry telemetry) {
        tele = telemetry;
        Drivetrain.init(hardwareMap);
        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        Arm.init(hardwareMap);
        Hopper.init(hardwareMap);
        VoltageSensor.init(hardwareMap);
        drive = new MyMecanumDrive(hardwareMap);
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }
    public static void initAuto(HardwareMap hardwareMap, Telemetry telemetry){
        Camera.init(hardwareMap, telemetry);
        init(hardwareMap, telemetry);
        Hopper.back();
        Arm.close();
        Camera.out();
        drive.setPoseEstimate(Coordinates.start);
        myLocalizer.setPoseEstimate(Coordinates.start);
        telemetry.addLine();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }
    public static void initTeleOp(HardwareMap hardwareMap, Telemetry telemetry){
        init(hardwareMap, telemetry);
        drive.setPoseEstimate(Coordinates.end);
        myLocalizer.setPoseEstimate(Coordinates.end);
        Hopper.back();
        Arm.open();
        telemetry.addLine();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }
    public static void initTest(HardwareMap hardwareMap, Telemetry telemetry){
        init(hardwareMap, telemetry);
        Hopper.back();
        Arm.close();
        drive.setPoseEstimate(Coordinates.start);
        myLocalizer.setPoseEstimate(Coordinates.start);
        telemetry.addLine();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }
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
    public static void update(Telemetry telemetry){
        telemetry.update();
        Robot.drive.update();
        Robot.myLocalizer.update();
        Hopper.back();
        Drivetrain.reportPose();
    }
}

