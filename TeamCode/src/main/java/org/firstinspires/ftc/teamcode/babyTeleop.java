package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    static double slidePower;
    static double actPower;
    static double actPower2;
    static double slowamount = 1;


    @Override
    public void runOpMode() {
    robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Drive Code

            fwdBackPower = gamepad1.left_stick_y*.5;
            strafePower = gamepad1.left_stick_x*.5;
            turnPower = gamepad1.right_stick_x*.30;
           // actPower = gamepad2.left_stick_y;
            // actPower2 = gamepad2.right_stick_y;


            lfPower = fwdBackPower - turnPower - strafePower;
            rfPower = fwdBackPower + turnPower + strafePower;
            lbPower = fwdBackPower - turnPower + strafePower;
            rbPower = fwdBackPower + turnPower - strafePower; 

            if (gamepad1.a == true){
                lfPower = lfPower*.1;
                lbPower = lbPower*.1;
                rfPower = rfPower*.1;
                rbPower = rbPower*.1;
            } else{
                break;
            }

            robot.leftfrontDrive.setPower(lfPower*slowamount);
            robot.leftbackDrive.setPower(lbPower*slowamount);
            robot.rightfrontDrive.setPower(rfPower*slowamount);
            robot.rightbackDrive.setPower(rbPower*slowamount);


           /*robot.linearActuator.setPower(actPower);
           robot.linearActuator2.setPower(-actPower2);
            if (gamepad2.a){
                robot.linearActuator.setPower(1);
                robot.linearActuator2.setPower(-1);
            } else if (gamepad2.b){
                robot.linearActuator.setPower(-1);
                robot.linearActuator2.setPower(1);
            } else{
                robot.linearActuator.setPower(0);
                robot.linearActuator2.setPower(0);
            }*/

            /*if (gamepad2.a) {
                robot.slideMotor.setPower(1);   //Extend
                robot.slideMotor2.setPower(-1);
            } else if (gamepad2.b) {
                robot.slideMotor.setPower(-1);     //Retract
                robot.slideMotor2.setPower(1);
            } else {
                robot.slideMotor.setPower(0);     //Stop Moving (Brake)
                robot.slideMotor2.setPower(0);
                }*/
            robot.slideMotor.setPower(-gamepad2.left_stick_y);
            robot.slideMotor2.setPower(gamepad2.left_stick_y);

           robot.intakeServo.setPower(gamepad2.right_stick_y);
    }
}}
