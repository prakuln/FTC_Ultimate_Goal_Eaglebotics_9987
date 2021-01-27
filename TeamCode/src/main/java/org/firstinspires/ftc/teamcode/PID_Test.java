package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PID_Test", group = "")
public class PID_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }
    /*while True:
   current_time = get_current_time()
   current_error = desire_position-current_position

   p = k_p * current_error

   i += k_i * (current_error * (current_time - previous_time))

   if i > max_i:
       i = max_i
   else if i < -max_i:
       i = -max_i

   D = k_d * (current_error - previous_error) / (current_time - previous_time)

   output = p + i + d

   previous_error = current_error
   previous_time = current_time
   */
}
