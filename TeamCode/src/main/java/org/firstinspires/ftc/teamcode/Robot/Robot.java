package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mecanum_Drive.MyMecanumDrive;
import org.firstinspires.ftc.teamcode.Mecanum_Drive.StandardTrackingWheelLocalizer;
public class Robot {
    public static MyMecanumDrive drive;
    public static StandardTrackingWheelLocalizer myLocalizer;

    private static void init(HardwareMap hardwareMap) {
        Drivetrain.init(hardwareMap);
        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        Arm.init(hardwareMap);
        VoltageSensor.init(hardwareMap);
        drive = new MyMecanumDrive(hardwareMap);
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }
    public static void initAuto(HardwareMap hardwareMap){
        Camera.init(hardwareMap);
        init(hardwareMap);
        Hopper.back();
        Arm.close();
        Camera.out();
        drive.setPoseEstimate(Coordinates.start);
        myLocalizer.setPoseEstimate(Coordinates.start);
        Camera.setZoom();
    }
    public static void initTeleOp(HardwareMap hardwareMap){
        init(hardwareMap);
        drive.setPoseEstimate(Coordinates.end);
        myLocalizer.setPoseEstimate(Coordinates.end);
        Camera.in();
        Hopper.back();
        Arm.open();
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
}

