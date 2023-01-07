package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto_Drive_Forward",group="AnyCorner")
public class Auto_Drive_Forward extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();

        waitForStart();
        encoderDrive(DRIVE_SPEED, 16,16,10,0);
    }
}
