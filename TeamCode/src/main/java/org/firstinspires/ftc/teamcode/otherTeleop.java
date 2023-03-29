package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name= "GIRAFFE", group= "Pushbot")

public class otherTeleop extends LinearOpMode{

    //speed variables here!!
    static double actpower;
    static double open = 1;
    static double closed = 0;

    otherHwMap robot = new otherHwMap();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){

    waitForStart();
    while (opModeIsActive()){

    robot.init(hardwareMap);
        robot.leftfrontDrive.setPower(gamepad1.right_stick_y);
        robot.leftbackDrive.setPower(gamepad1.right_stick_y);
        robot.rightfrontDrive.setPower(-gamepad1.left_stick_y);
        robot.rightbackDrive.setPower(-gamepad1.left_stick_y);

     if(gamepad1.y){
         robot.linearActuator.setPower(1);
     }
     else if(gamepad1.x){
         robot.linearActuator.setPower(-1);
     }
     else{
         robot.linearActuator.setPower(0);
     }
        if(gamepad1.a){
            robot.linearActuator2.setPower(1);
        }
        else if (gamepad1.b){
            robot.linearActuator2.setPower(-1);
        }
        else{
            robot.linearActuator2.setPower(0);
        }

        if (gamepad1.left_bumper) {
            robot.openServo.setPower(1);
        }
        else if (gamepad1.right_bumper){
            robot.openServo.setPower(-1);
        }
        else{
            robot.openServo.setPower(-0);
        }

    }



}}
