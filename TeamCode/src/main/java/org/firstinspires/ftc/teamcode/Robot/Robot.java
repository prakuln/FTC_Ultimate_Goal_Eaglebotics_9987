//The class is packaged in org.firstinspires.ftc.teamcode.Robot
package org.firstinspires.ftc.teamcode.Robot;

//HardwareMap provides a means of retrieving runtime HardwareDevice instances according to the names with which the corresponding physical devices were associated during robot configuration.
import com.qualcomm.robotcore.hardware.HardwareMap;

//Instances of Telemetry provide a means by which data can be transmitted from the robot controller to the driver station and displayed on the driver station screen.
import org.firstinspires.ftc.robotcore.external.Telemetry;

//This class extends extends com.acmerobotics.roadrunner.drive.MecanumDrive and has method to getWheelPositions, getWheelVelocities etc
import org.firstinspires.ftc.teamcode.Mecanum_Drive.MyMecanumDrive;

//This class extends ThreeTrackingWheelLocalizer and overrides methods like getWheelPositions, getWheelVelocities
import org.firstinspires.ftc.teamcode.Mecanum_Drive.StandardTrackingWheelLocalizer;

//This class defines different coordinates like start, shoot, wooble etc
import org.firstinspires.ftc.teamcode.Trajectories.Coordinates;

//This class defines view and has method like getHeight
import org.firstinspires.ftc.teamcode.Vision.Camera;

//Robot class initilizes all the hardware and robot components and then has a method to update those components
public class Robot {
    public static MyMecanumDrive drive;
    public static StandardTrackingWheelLocalizer myLocalizer;
    public static Telemetry tele;

    //Init method helps to initialize Drivetrain, shooter, Intake, Arm, Hopper, VoltageSensor....
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
    
    //initAuto method helps to initialize all from init method plus Camera and  Arm, Hopper, VoltageSensor....
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
    //initTeleOp method helps to initialize all from Init method plus drive, myLocalizer, Hopper and Arm....
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
    
     //initTest method helps to test init, plus Hooper, Arm, Drive....
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
    
     //wait method helps to sleep for millisec
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

    //update method helps to call telemetry update, drive update, mylocalizer update....
    public static void update(Telemetry telemetry){
        telemetry.update();
        Robot.drive.update();
        Robot.myLocalizer.update();
        Hopper.back();
        Drivetrain.reportPose();
    }
}

