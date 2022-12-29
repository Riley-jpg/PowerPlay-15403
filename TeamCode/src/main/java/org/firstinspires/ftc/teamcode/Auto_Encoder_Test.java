package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAutonomousWithEncoder",group="EncoderTest")
public class Auto_Encoder_Test extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        encoderDrive(DRIVE_SPEED, 17,17,10,0);
        encoderStrafe(STRAFE_SPEED,30,30,10,0);
        encoderLift(LIFT_SPEED, 28, 28, 10, 0);
        encoderDrive(DRIVE_SPEED, 1, 1, 10, 0);
        encoderLift(LIFT_SPEED, -1, -1, 10, 0);
        while(runtime.seconds() < 5) {
            intakeServo.setPower(-1);
        }
        intakeServo.setPower(0);
        encoderLift(LIFT_SPEED, -27, -27, 10, 0);
        encoderDrive(DRIVE_SPEED, -1, -1, 10, 0);
        encoderStrafe(STRAFE_SPEED, -29, -29, 10, 0 );
    }
}
