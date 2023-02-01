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

@TeleOp (name= "other teleop", group= "Pushbot")
@Disabled
public class otherTeleop extends LinearOpMode{

    //speed variables here!!
    static double actpower;
    static double open = 1;
    static double closed = 0;

    otherHwMap robot = new otherHwMap();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
    robot.init(hardwareMap);
        robot.leftfrontDrive.setPower(gamepad1.left_stick_y);
        robot.leftbackDrive.setPower(gamepad1.left_stick_y);
        robot.rightfrontDrive.setPower(gamepad1.right_stick_y);
        robot.rightbackDrive.setPower(gamepad1.right_stick_y);

        robot.linearActuator.setPower(gamepad2.right_stick_y);
        robot.linearActuator2.setPower(gamepad2.left_stick_y);

        if (gamepad2.a) {
            robot.openServo.setPosition(open);
        }
        else if (gamepad2.b){
            robot.openServo.setPosition(closed);
        }

        telemetry.addData("servo positon", robot.openServo.getPosition());
        telemetry.update();
    }



}
