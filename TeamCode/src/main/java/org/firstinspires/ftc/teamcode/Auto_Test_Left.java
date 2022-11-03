package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Basic Drive Test Left Side of the Field for Autonomous
 */
@Autonomous(name="TestAutonomousWithoutVuforia",group="Left")
public class Auto_Test_Left extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        setAllDriveMotors(7);
        strafeRight(7);
        strafeLeft(7);
    }
}