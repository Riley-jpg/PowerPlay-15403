package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Basic Drive Test Right Side of Field for Autonomous
 */
@Autonomous(name="TestAutonomousWithoutVuforia",group="Right")
public class Auto_Test_Right extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        setAllDriveMotors(7);
        strafeLeft(7);
        strafeRight(7);
    }
}
