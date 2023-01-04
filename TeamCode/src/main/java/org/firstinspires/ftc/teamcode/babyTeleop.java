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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name= "babyTeleop", group= "Pushbot")

public class babyTeleop extends Auto_Util {

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
    static double liftPower;


    @Override
    public void runOpMode() {
    robot.init(hardwareMap);
        initAuto();
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Drive Code

            fwdBackPower =- gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            turnPower =- gamepad1.right_stick_x*.30;
            liftPower = gamepad2.left_stick_y;
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

            }

            robot.leftfrontDrive.setPower(lfPower*slowamount);
            robot.leftbackDrive.setPower(lbPower*slowamount);
            robot.rightfrontDrive.setPower(rfPower*slowamount);
            robot.rightbackDrive.setPower(rbPower*slowamount);

            if(gamepad1.right_bumper){
                slowamount = 0.5;
            } else {
                slowamount = 1;
            }

           robot.slideMotor.setPower(-liftPower);
           robot.slideMotor2.setPower(liftPower);


            robot.intakeServo.setPower(gamepad2.right_stick_y);

            if (gamepad2.a){
                encoderLift(1, 13, 13, 10, 0);}
            if (gamepad2.b) {
                encoderLift(1, 21, 21, 10, 0);}
            if (gamepad2.y) {
                encoderLift(1, 35, 35, 10, 0);}
            if(gamepad2.x){
                encoderLift(1, -35, -35, 10, 0);}


            //Unused Code Graveyard:
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

           telemetry.addData("slidePosit", robot.slideMotor.getCurrentPosition());
            telemetry.addData("slide2Posit", robot.slideMotor2.getCurrentPosition());
            telemetry.update();
    }
}}
