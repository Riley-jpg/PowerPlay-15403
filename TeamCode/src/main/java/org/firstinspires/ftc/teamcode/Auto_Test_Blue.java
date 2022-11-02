package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Basic Drive Test for Autonomous
 */
@Autonomous(name="TestAutonomousWithoutVuforia",group="Blue")
public class Auto_Test_Blue extends Auto_Util {
    public void runOpMode() throws InterruptedException {
    waitForStart();
    setAllDriveMotors(5);
    strafeRight(5);
    strafeLeft(5);
    }
}