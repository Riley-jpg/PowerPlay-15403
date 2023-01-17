package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.babyHwMap;

import java.util.List;

@Autonomous(name="TensorFlowDriveTest", group="TensorFlowTest")
public class TensorFlowDriveTest extends Auto_Util{
    public void runOpMode() throws InterruptedException {
        initAuto();

        waitForStart();
        String objectDetected = useVision();
        encoderDrive(DRIVE_SPEED, 16,16,10,0);
        if(objectDetected == "Bolt"){
            encoderStrafe(STRAFE_SPEED, -10, -10, 10, 0);
        }
        else if (objectDetected == "Bulb"){
        }
        else if (objectDetected == "Panel"){
            encoderStrafe(STRAFE_SPEED, 10, 10, 10, 0);
        }
        else {
            telemetry.addData("Value:", objectDetected);
            telemetry.update();
        }
    }
}