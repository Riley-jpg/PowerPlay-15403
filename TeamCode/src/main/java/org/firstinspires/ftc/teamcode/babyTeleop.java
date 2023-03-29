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

@TeleOp (name= "teenageTeleop", group= "Pushbot")

public class babyTeleop extends Auto_Util {

    babyHwMap robot = new babyHwMap();
    private ElapsedTime runtime = new ElapsedTime();
    static double turnPower ;
    static double fwdBackPower;
    static double strafePower;
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

            //Drive

            fwdBackPower = -gamepad1.left_stick_y*.7;
            strafePower = -gamepad1.left_stick_x*.7;
            turnPower = -gamepad1.right_stick_x*.8;
            liftPower = gamepad2.left_stick_y;
           // actPower = gamepad2.left_stick_y;
            // actPower2 = gamepad2.right_stick_y;


            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);



            //slow mode stuff

            robot.leftfrontDrive.setPower(lfPower*slowamount);
            robot.leftbackDrive.setPower(lbPower*slowamount);
            robot.rightfrontDrive.setPower(rfPower*slowamount);
            robot.rightbackDrive.setPower(rbPower*slowamount);

           if (gamepad1.right_bumper){
                slowamount = 0.5;
            } else if(gamepad1.left_bumper) {
                slowamount = 0.1;
            }else{
                slowamount = 1;
            }
            //slides stuff

           robot.slideMotor.setPower(-liftPower);
           robot.slideMotor2.setPower(liftPower);


            robot.intakeServo.setPower(-gamepad2.right_stick_y);

            //slide encoders possibly???? idk what i'm doing here
            if(gamepad1.a){
                robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            robot.slideMotor.setTargetPosition(0);
            robot.slideMotor2.setTargetPosition(0);

            if(gamepad2.right_trigger > .1){
                robot.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            //indiv wheels for troubleshooting
            if(gamepad1.dpad_up){
                robot.leftfrontDrive.setPower(1);
            }
            if(gamepad1.dpad_right){
                robot.rightfrontDrive.setPower(1);
            }
            if(gamepad1.dpad_down){
                robot.leftbackDrive.setPower(1);
            }
            if(gamepad1.dpad_left){
                robot.rightbackDrive.setPower(1);
            }





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

           /* if (gamepad2.a){
                encoderLift(1, 13 , 10, 0);}
            if (gamepad2.b) {
                encoderLift(1, 21, 10, 0);}
            if (gamepad2.y) {
                encoderLift(1, 35, 10, 0);}
            if(gamepad2.x){
                encoderLift(1, -35, 10, 0);}*/



           telemetry.addData("lfD", robot.leftfrontDrive.getCurrentPosition());
            telemetry.addData("rfD", robot.rightfrontDrive.getCurrentPosition());
            telemetry.addData("lbD", robot.leftbackDrive.getCurrentPosition());
            telemetry.addData("rbD", robot.rightbackDrive.getCurrentPosition());
            telemetry.addData("rs", robot.slideMotor.getCurrentPosition());
            telemetry.addData("ls", robot.slideMotor2.getCurrentPosition());

            telemetry.update();
    }
}}
