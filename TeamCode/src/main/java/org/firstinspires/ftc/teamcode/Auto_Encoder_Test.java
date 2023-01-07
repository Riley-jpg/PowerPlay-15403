package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAutonomousWithEncoder",group="EncoderTest")
public class Auto_Encoder_Test extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();

        waitForStart();
        encoderDrive(DRIVE_SPEED, 16,16,10,0);
        encoderStrafe(STRAFE_SPEED,8.5,8.5,10,0); //maybe 12 for high pole
        slideLift(5);
        encoderDrive(DRIVE_SPEED, 2, 2, 10, 0);
        slideLower(1);
        while(runtime.seconds() < 2) {
            intakeServo.setPower(-1);
        }
        intakeServo.setPower(0);
        encoderDrive(DRIVE_SPEED, -2, -2, 10, 0);
        encoderStrafe(STRAFE_SPEED, -8.5, -8.5, 10, 0 );
    }
}
