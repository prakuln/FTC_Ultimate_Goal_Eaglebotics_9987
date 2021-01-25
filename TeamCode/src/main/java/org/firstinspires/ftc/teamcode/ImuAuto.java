package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
@TeleOp(name = "ImuAuto", group = "")
public class ImuAuto extends LinearOpMode {
    Orientation angles;
    private BNO055IMU imu;
    boolean bool = true;
    int initialHeading, leftHeading, rightHeading;
    private DcMotor MRight;
    private DcMotor MLeft;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        MRight = hardwareMap.dcMotor.get("MRight"); //initialize the motors
        MLeft = hardwareMap.dcMotor.get("MLeft");
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        wait(100);
        initialHeading = getAngle();
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
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Get absolute orientation

                if (gamepad1.x){
                    correctLeft();

                }
                if (gamepad1.b){
                    correctRight();

                }
                if (gamepad1.y){
                    correct180();

                }
                telemetry.addData("Live Heading: ", getAngle());
                telemetry.update();
            }
        }

    }
    private int getAngle()
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
    public void correctLeft(){
        while(getAngle()!= leftHeading){
            if(getAngle()< leftHeading){
                MLeft.setPower(0.3);
                MRight.setPower(0.3);
            } else if(getAngle() > leftHeading) {
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
    public void correctRight(){
        while(getAngle()!= rightHeading){
            if(getAngle()< rightHeading){
                MLeft.setPower(0.3);
                MRight.setPower(0.3);
            } else if(getAngle() > rightHeading) {
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