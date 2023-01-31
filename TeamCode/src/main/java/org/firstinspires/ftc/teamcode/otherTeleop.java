package org.firstinspires.ftc.teamcode;

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
public class otherTeleop extends otherHwMap{

    //speed variables here!!

    otherHwMap robot = new otherHwMap();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){
        if(gamepad1.left)

    }



}
