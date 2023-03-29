package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto_Drive_Forward",group="AnyCorner")
public class Auto_Drive_Forward extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();

        waitForStart();

        /*encoderDrive(0.1,4,4,20,0);
        encoderStrafe(0.1,13,13,20,0);
        encoderDrive(0.2, 31,31,20,0);
        encoderStrafe(0.1,-7.75,-7.75,20,0);
        encoderLift(0.75,75,20,0);
        encoderDrive(0.08,2,2,20,0);
        encoderLift(0.5,-5,20,0);
        intakeServo.setPower(1);
        sleep(1500);
        intakeServo.setPower(0);
        encoderDrive(0.2,-5,-5,20,0);
        encoderLift(1,-60,20,0);
        sleep(5000);

         */
        encoderDrive(0.5, -9, 9, 300, 0);

    }
}
