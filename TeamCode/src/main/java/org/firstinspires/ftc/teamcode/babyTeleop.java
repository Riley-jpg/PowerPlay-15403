package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name= "babyTeleop", group= "Pushbot")

public class babyTeleop extends LinearOpMode {

    babyHwMap robot = new babyHwMap();
    private ElapsedTime runtime = new ElapsedTime();
    static double turnPower = 0.5;
    static double fwdBackPower = 0.5;
    static double strafePower = 0.5;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount = 1;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            fwdBackPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = -gamepad1.right_stick_x;

            lfPower = fwdBackPower - turnPower - strafePower;
            rfPower = fwdBackPower + turnPower + strafePower;
            lbPower = fwdBackPower - turnPower + strafePower;
            rbPower = fwdBackPower + turnPower - strafePower;

            robot.leftfrontDrive.setPower(-lfPower*slowamount);
            robot.leftbackDrive.setPower(-lbPower*slowamount);
            robot.rightfrontDrive.setPower(-rfPower*slowamount);
            robot.rightbackDrive.setPower(-rbPower*slowamount);

        }
    }
}
