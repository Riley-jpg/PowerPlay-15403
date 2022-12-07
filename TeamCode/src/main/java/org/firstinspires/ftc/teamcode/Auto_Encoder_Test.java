package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAutonomousWithEncoder",group="EncoderTest")
public class Auto_Encoder_Test extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        encoderDrive(DRIVE_SPEED, 5,5,10,0);
        encoderStrafe(STRAFE_SPEED,5,5,10,0);
        encoderLift(LIFT_SPEED, 5, 5, 10, 0);
        encoderStrafe(STRAFE_SPEED, -5, -5, 10, 0 );
    }
}
