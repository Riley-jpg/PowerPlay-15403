/*
leftDrive is port 0
rightDrive is port 1
ArmMotor is port 2
IntakeMotor is port 3

ArmServo is 0
Intake Servo is 1

Expansion Hub:
Duck is port 2







 */
package org.firstinspires.ftc.teamcode;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Auto_Util", group="abstract")
@Disabled
public abstract class Auto_Util extends LinearOpMode{
    /*
    ___________________________________________________________________________________________________________________________________
    -
    -VARIABLES!
    -
    ___________________________________________________________________________________________________________________________________
     */
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.75;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double ENCODER_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.1;
    static final double STRAFE_SPEED = 0.4;
    static final double TARGET_SHOOTER_SPEED = 2;

    //Vision Colors
    int maxAll = getColorInt(255, 255, 255, 255);
    int minAll = getColorInt(255, 0, 0, 0);
    int maxRed = getColorInt(255, 255, 150, 150);
    int minRed = getColorInt(255, 150, 0, 0);
    int maxBlue = getColorInt(255, 150, 255, 255);
    int minBlue = getColorInt(255, 100, 100, 100);
    int maxCap = getColorInt(255, 92, 255, 228);
    int minCap = getColorInt(255, 25, 181, 155);

    public static final int RED = 1;
    public static final int BLUE = 2;
    public static final int CAP = 3;

    //Drive motors
    DcMotor rfmotor, rbmotor, lfmotor, lbmotor;
    //Utility motors
    DcMotor utilmotor1, utilmotor2, utilmotor3, utilmotor4;
    //odometry encoders
    DcMotor verticalLeft, verticalRight, horizontal;
    //servos
    Servo servo1;
    CRServo crservo1, crservo2;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    static double motor_power;
    //Hardware Map Names for drive motors and odometry wheels. This may need to be changed between years if the config changes
    String rfName = "rfD", rbName = "rbD", lfName = "lfD", lbName = "lbD";
    String util1name = "Intake", util2name = "pastaM", util3name = "shootM", util4name = "wobbleG";
    String servo1name = "wobbleS", crservo1name = "pastaS", crservo2name = "pastaS2";
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    //Variables for Camera
    private static final String TAG = "Webcam Sample";
    private static final int secondsPermissionTimeout = 100;
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private Handler callbackHandler;
    public boolean useTwoRegions = false;
    int aMinX = 1;
    int aMaxX = 640;
    int aMinY = 1;
    int aMaxY = 480;
    //
    int bMinX = 1;
    int bMaxX = 640;
    int bMinY = 1;
    int bMaxY = 480;
    //
    int cMinX = 1;
    int cMaxX = 640;
    int cMinY = 1;
    int cMaxY = 480;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("you shouldn't be here!", "This program isnt meant to be run, only for use with all of its methods");
        telemetry.update();
    }
    /*
    ___________________________________________________________________________________________________________________________________
    -
    -ALL OF THE HARDWARE AND HARDWARE INITIALIZATION METHODS
    -
    ___________________________________________________________________________________________________________________________________
     */
    public void initAuto(){
        initDriveHardwareMap(rfName, rbName, lfName, lbName);
        initUtilHardwareMap(util1name, util2name, util3name, util4name);
        initServoHardwareMap(servo1name, crservo1name, crservo2name);
        //IMU Stuff, sets up parameters and reports accelerations to logcat log
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmodeaz
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void assignDriveBase(DcMotor rightfrontmotor, DcMotor rightbackmotor, DcMotor leftfrontmotor, DcMotor leftbackmotor){
        rfmotor = rightfrontmotor; rbmotor = rightbackmotor; lfmotor = leftfrontmotor; lbmotor = leftbackmotor;
    }
    public void assignUtilMotors(DcMotor util1, DcMotor util2, DcMotor util3, DcMotor util4){
        utilmotor1 = util1; utilmotor2 = util2; utilmotor3 = util3; utilmotor4 = util4;
    }
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName){
        rfmotor = hardwareMap.dcMotor.get(rfName); rfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor = hardwareMap.dcMotor.get(rbName); rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfmotor = hardwareMap.dcMotor.get(lfName); lfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbmotor = hardwareMap.dcMotor.get(lbName); lbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); rfmotor.setDirection(DcMotor.Direction.FORWARD); rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); rbmotor.setDirection(DcMotor.Direction.FORWARD); rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); lfmotor.setDirection(DcMotor.Direction.REVERSE); lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); lbmotor.setDirection(DcMotor.Direction.REVERSE); lbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initUtilHardwareMap(String util1name, String util2name, String util3name, String util4name){
        utilmotor1 = hardwareMap.dcMotor.get(util1name); utilmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor2 = hardwareMap.dcMotor.get(util2name); utilmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor3 = hardwareMap.dcMotor.get(util3name); utilmotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor4 = hardwareMap.dcMotor.get(util4name); utilmotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        utilmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); utilmotor1.setDirection(DcMotor.Direction.FORWARD); utilmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); utilmotor2.setDirection(DcMotor.Direction.FORWARD); utilmotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); utilmotor3.setDirection(DcMotor.Direction.FORWARD); utilmotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER); utilmotor4.setDirection(DcMotor.Direction.FORWARD); utilmotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //THIS ONE IS YEAR SPECIFIC. WE MAY HAVE MORE OR LESS SERVOS AND CONTINUOUS ROTATION SERVOS THAN THIS
    private void initServoHardwareMap(String servo1name, String crservo1name, String crservo2name){
        servo1 = hardwareMap.servo.get(servo1name); servo1.setPosition(0);
        crservo1 = hardwareMap.crservo.get(crservo1name); crservo1.setDirection(CRServo.Direction.FORWARD); crservo1.setPower(0);
        crservo2 = hardwareMap.crservo.get(crservo2name); crservo2.setDirection(CRServo.Direction.FORWARD); crservo2.setPower(0);
    }
    /*
   ___________________________________________________________________________________________________________________________________
   -
   -ALL OF THE ENCODER DRIVING METHODS
   -
   ___________________________________________________________________________________________________________________________________
    */
    public static double heading(BNO055IMU imu) {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    public double PI(double desiredHeading) {
        double error = Math.abs(desiredHeading - Math.abs(heading(imu)));
        double Kp = 0.01;
        if (error > 2.0 || error < -2.0) {
            return Kp * error;
        }
        else {
            return 0;
        }
    }
    public double accelerate(DcMotor motor, double speed, double target){
        if(motor.getCurrentPosition()<(target/10)){
            return speed*1.30;
        }
        else if(motor.getCurrentPosition()>(target*8/10)){
            return speed*0.9;
        }
        return speed;
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS , double desiredHeading) {
        int leftBackTarget;
        int rightBackTarget;
        int rightFrontTarget;
        int leftFrontTarget;
        //int averageTarget;
        double leftSpeed, rightSpeed;
        if (opModeIsActive()) {
            if(leftInches < 0){
                leftSpeed = speed*-1;
            }
            else {
                leftSpeed = speed;
            }
            if(rightInches < 0){
                rightSpeed = speed*-1;
            }
            else {
                rightSpeed = speed;
            }
            resetEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget =  (lbmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightBackTarget = (rbmotor.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            leftFrontTarget = (lfmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightFrontTarget = (rfmotor.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            lfmotor.setTargetPosition(leftFrontTarget);
            lbmotor.setTargetPosition(leftBackTarget);
            rfmotor.setTargetPosition(rightFrontTarget);
            rbmotor.setTargetPosition(rightBackTarget);



            // Turn On RUN_TO_POSITION
            rfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rbmotor.setPower(0.7*(rightSpeed + PI(desiredHeading)));
            rfmotor.setPower(0.7*(rightSpeed + PI(desiredHeading)));
            lfmotor.setPower(0.7*(leftSpeed - PI(desiredHeading)));
            lbmotor.setPower(0.7*(leftSpeed -  PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&(rbmotor.isBusy()) && (rfmotor.isBusy()) && (lbmotor.isBusy()) && (lfmotor.isBusy())) {
                telemetry.addData("Left Back Current Position", lbmotor.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rbmotor.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", lfmotor.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rfmotor.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                //telemetry.addData("rightSpeed",rightSpeed);
                //telemetry.addData("leftSpeed",leftSpeed);
                telemetry.addData("heading", heading(imu));
                telemetry.update();
                leftSpeed = (accelerate(lbmotor,leftSpeed,leftBackTarget)+accelerate(lfmotor,leftSpeed,leftFrontTarget)/2);
                rightSpeed = (accelerate(rbmotor,rightSpeed,rightBackTarget)+accelerate(rfmotor,rightSpeed,rightFrontTarget)/2);
                rbmotor.setPower((rightSpeed + PI(desiredHeading)));
                rfmotor.setPower((rightSpeed + PI(desiredHeading)));
                lfmotor.setPower((leftSpeed - PI(desiredHeading)));
                lbmotor.setPower((leftSpeed -  PI(desiredHeading)));
            }

            lbmotor.setPower(0);
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            rbmotor.setPower(0);
            sleep(100);
        }
    }
    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS , double desiredHeading) {
        int leftBackTarget;
        int rightBackTarget;
        int rightFrontTarget;
        int leftFrontTarget;
        //int averageTarget;
        double leftSpeed, rightSpeed;
        if (opModeIsActive()) {
            if(leftInches < 0){
                leftSpeed = speed*-1;
            }
            else {
                leftSpeed = speed;
            }
            if(rightInches < 0){
                rightSpeed = speed*-1;
            }
            else {
                rightSpeed = speed;
            }
            resetEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget =  (lbmotor.getCurrentPosition() - (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightBackTarget = (rbmotor.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            leftFrontTarget = (lfmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightFrontTarget = (rfmotor.getCurrentPosition() - (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            lfmotor.setTargetPosition(leftFrontTarget);
            lbmotor.setTargetPosition(leftBackTarget);
            rfmotor.setTargetPosition(rightFrontTarget);
            rbmotor.setTargetPosition(rightBackTarget);



            // Turn On RUN_TO_POSITION
            rfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rbmotor.setPower(0.7*(rightSpeed + PI(desiredHeading)));
            rfmotor.setPower(0.7*(rightSpeed + PI(desiredHeading)));
            lfmotor.setPower(0.7*(leftSpeed - PI(desiredHeading)));
            lbmotor.setPower(0.7*(leftSpeed -  PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&(rbmotor.isBusy()) && (rfmotor.isBusy()) && (lbmotor.isBusy()) && (lfmotor.isBusy())) {
                telemetry.addData("Left Back Current Position", lbmotor.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rbmotor.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", lfmotor.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rfmotor.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                telemetry.addData("heading", heading(imu));
                //telemetry.addData("Average Target",averageTarget);
                telemetry.addData("rightSpeed",rightSpeed);
                telemetry.addData("leftSpeed",leftSpeed);
                telemetry.update();
                leftSpeed = (accelerate(lbmotor,leftSpeed,leftBackTarget)+accelerate(lfmotor,leftSpeed,leftFrontTarget)/2);
                rightSpeed = (accelerate(rbmotor,rightSpeed,rightBackTarget)+accelerate(rfmotor,rightSpeed,rightFrontTarget)/2);
                rbmotor.setPower((rightSpeed + PI(desiredHeading)));
                rfmotor.setPower((rightSpeed + PI(desiredHeading)));
                lfmotor.setPower((leftSpeed - PI(desiredHeading)));
                lbmotor.setPower((leftSpeed -  PI(desiredHeading)));
            }

            lbmotor.setPower(0);
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            rbmotor.setPower(0);
            sleep(100);
        }
    }
    public void resetEncoders(){
        lfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
    ___________________________________________________________________________________________________________________________________
    -
    -GAME SPECIFIC METHODS! SCRAP AFTER THIS YEAR!
    -
    ___________________________________________________________________________________________________________________________________
     */
    public void shoot(double time){
        runtime.reset();
        utilmotor3.setPower(motor_power);
        sleep(900);
        while(runtime.seconds() < time){
            utilmotor1.setPower(-1); utilmotor2.setPower(-1);
            utilmotor3.setPower(-1); crservo1.setPower(1);
            crservo2.setPower(-1);
        }
        utilmotor1.setPower(0); utilmotor2.setPower(0);
        utilmotor3.setPower(0); crservo1.setPower(0);
        crservo2.setPower(0);
    }
    public void smartShoot (double time){
        double current_speed;
        double numberofRevolutions;
        double velocityoffset;
        runtime.reset();
        while(runtime.seconds() < time){
            if(runtime.seconds() > 4){
                utilmotor1.setPower(-1);
                utilmotor2.setPower(-1);
                crservo1.setPower(1);
                crservo2.setPower(-1);
            }
            numberofRevolutions = utilmotor3.getCurrentPosition()/COUNTS_PER_MOTOR_REV;
            current_speed = numberofRevolutions/runtime.seconds();
            velocityoffset = -TARGET_SHOOTER_SPEED-current_speed;
            velocityoffset = velocityoffset*1.6;

            motor_power = -Math.abs(-1 + velocityoffset);
            utilmotor3.setPower(motor_power);
            telemetry.addData("Encoder Value", utilmotor3.getCurrentPosition());
            telemetry.addData("Current Speed", current_speed);
            telemetry.addData("Velocity offset", velocityoffset);
            telemetry.addData("Motor Power", motor_power);
            telemetry.update();

            //utilmotor3.setPower(-1);

        }
        utilmotor1.setPower(0); utilmotor2.setPower(0);
        utilmotor3.setPower(0); crservo1.setPower(0);
        crservo2.setPower(0);
    }
    /*
    ___________________________________________________________________________________________________________________________________
    -
    -BASIC (UNTESTED) DRIVE BY TIME METHODS. THESE SHOULD HELP WITH DEBUGGING ONCE THEY, Y'KNOW, GET DEBUGGED
    -
    ___________________________________________________________________________________________________________________________________
     */
    public void setAllDriveMotors(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(1); rbmotor.setPower(1);
            lfmotor.setPower(1); lbmotor.setPower(1);
        }
        rfmotor.setPower(0); rbmotor.setPower(0);
        lfmotor.setPower(0); lbmotor.setPower(0);
    }
    public void strafeLeft(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(1); rbmotor.setPower(-1);
            lfmotor.setPower(-1); lbmotor.setPower(1);
        }
        rfmotor.setPower(0); rbmotor.setPower(0);
        lfmotor.setPower(0); lbmotor.setPower(0);
    }
    public void strafeRight(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(-1); rbmotor.setPower(1);
            lfmotor.setPower(1); lbmotor.setPower(-1);
        }
        rfmotor.setPower(0); rbmotor.setPower(0);
        lfmotor.setPower(0); lbmotor.setPower(0);
    }
    public void turnRight(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(-0.5); rbmotor.setPower(-0.5);
            lfmotor.setPower(0.5); lbmotor.setPower(0.5);
            telemetry.addData("seconds or somethin", runtime.seconds());
            telemetry.update();
        }
        rfmotor.setPower(0); rbmotor.setPower(0);
        lfmotor.setPower(0); lbmotor.setPower(0);
    }
    public void turnLeft(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(1); rbmotor.setPower(1);
            lfmotor.setPower(-1); lbmotor.setPower(-1);
        }
        rfmotor.setPower(0); rbmotor.setPower(0);
        lfmotor.setPower(0); lbmotor.setPower(0);
    }
    /*
    ___________________________________________________________________________________________________________________________________
    -
    -VISION METHODS!
    -
    ___________________________________________________________________________________________________________________________________
     */
    public void useTwoRegions(boolean val) {
        useTwoRegions = val;
    }

    public void setVisionRegionsA(int minX, int maxX, int minY, int maxY) {
        aMinX = minX;
        aMaxX = maxX;
        aMinY = minY;
        aMaxY = maxY;
    }

    public void setVisionRegionB(int minX, int maxX, int minY, int maxY) {
        bMinX = minX;
        bMaxX = maxX;
        bMinY = minY;
        bMaxY = maxY;
    }

    public void setVisionRegionsC(int minX, int maxX, int minY, int maxY) {
        cMinX = minX;
        cMaxX = maxX;
        cMinY = minY;
        cMaxY = maxY;
    }

    public void initVision() {
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        openCamera();
        startCamera();
    }

    public int barcodeValue(Bitmap frameMap, int targetColor) {
        //Decide on color
        //Divide main bitmap into 3 subsets
        //Bitmap A
        //telemetry.addLine("Attempting to divide bitmap...");
        telemetry.update();
        int aHeight = aMaxY - aMinY;
        int aWidth = aMaxX - aMinX;
        Bitmap bitmapA = Bitmap.createBitmap(frameMap, aMinX, aMinY, aWidth, aHeight);
        if(bitmapA != null) {
            telemetry.addLine("bitmapA created.");
        }
        else {
            telemetry.addLine("Failed to create bitmapA");
        }
        //Bitmap B
        int bHeight = bMaxY - bMinY;
        int bWidth = bMaxX - bMinX;
        Bitmap bitmapB = Bitmap.createBitmap(frameMap, bMinX, bMinY, bWidth, bHeight);
        if(bitmapB != null) {
            telemetry.addLine("bitmapB created.");
        }
        else {
            telemetry.addLine("Failed to create bitmapB");
        }
        //Bitmap C
        int cHeight = cMaxY - cMinY;
        int cWidth = cMaxX - cMinX;
        Bitmap bitmapC = Bitmap.createBitmap(frameMap, cMinX, cMinY, cWidth, cHeight);
        if(bitmapC != null) {
            telemetry.addLine("bitmapC created.");
        }
        else {
            telemetry.addLine("Failed to create bitmapC");
        }

        telemetry.addLine("Bitmap divided. Attempting to count pixels...");
        telemetry.update();
        //Get how many pixels fall within target color for each bitmap
        //int aPixels = pixelsColor(bitmapA, targetColorMin, targetColorMax);
        int aPixels = newPixelsColorCount(bitmapA, targetColor);
        telemetry.addLine("aPixels has been counted.");
        //==========
        //sleep(10000);
        //==========
        int bPixels = newPixelsColorCount(bitmapB, targetColor);
        telemetry.addLine("bPixels has been counted.");
        int cPixels = newPixelsColorCount(bitmapC, targetColor);
        telemetry.addLine("cPixels has been counted.");

        telemetry.addLine("Pixels counted. Attempting to compare counts");
        telemetry.update();
        telemetry.addLine("A_Pixels: " + aPixels);
        telemetry.addLine("B_Pixels: " + bPixels);
        if(useTwoRegions) {
            telemetry.addLine("Total Pixels (c): " + cPixels);
        }
        else {
            telemetry.addLine("C_Pixels: " + cPixels);
        }
        telemetry.update();
        //    sleep(10000);
        if(useTwoRegions == true) {
            if(aPixels < 500 && aPixels < bPixels) {
                return 2;
            }
            else if(bPixels < 500 && bPixels < aPixels) {
                return 3;
            }
            else {
                return 1;
            }
        }
        else {
            if(aPixels > bPixels && aPixels > cPixels) {
                return 1;
            }
            else if(bPixels > aPixels && bPixels > cPixels) {
                return 2;
            }
            else if(cPixels > bPixels && cPixels > aPixels) {
                return 3;
            }
        }
        return 0;
    }

    public int getColorInt(int alphaVal, int redVal, int greenVal, int blueVal) {
        int combColor = (alphaVal & 0xff) << 24 | (redVal & 0xff) << 16 | (greenVal & 0xff) << 8 | (blueVal & 0xff);
        return combColor;
    }

    public int newPixelsColorCount(Bitmap frameMap, int color) {
        int pixelCount = 0;
        if(color == 1) {//RED
            int minR = red(minRed);
            int minG = green(minRed);
            int minB = blue(minRed);
            int maxR = red(maxRed);
            int maxG = green(maxRed);
            int maxB = blue(maxRed);
            for(int i = 1; i < frameMap.getHeight(); i++) {
                for(int j = 1; j < frameMap.getWidth(); j++) {
                    int curPixel = frameMap.getPixel(j, i);
                    int pR = red(curPixel);
                    int pG = green(curPixel);
                    int pB = blue(curPixel);
                    if(pR >= minR && pR <= maxR) {
                        if(pG >= minG && pG <= maxG) {
                            if(pB >= minB && pB <= maxB) {
                                pixelCount++;
                            }
                        }
                    }
                }
            }
        }
        else if(color == 2) {//BLUE
            int minR = red(minBlue);
            int minG = green(minBlue);
            int minB = blue(minBlue);
            int maxR = red(maxBlue);
            int maxG = green(maxBlue);
            int maxB = blue(maxBlue);
            for(int i = 1; i < frameMap.getHeight(); i++) {
                for(int j = 1; j < frameMap.getWidth(); j++) {
                    int curPixel = frameMap.getPixel(j, i);
                    int pR = red(curPixel);
                    int pG = green(curPixel);
                    int pB = blue(curPixel);
                    if(pR >= minR && pR <= maxR) {
                        if(pG >= minG && pG <= maxG) {
                            if(pB >= minB && pB <= maxB) {
                                if(pB > pR + 20 && pB > pG + 20) {
                                    pixelCount++;
                                }
                            }
                        }
                    }
                }
            }
        }
        else if(color == 3) {//CAP

        }
        //telemetry.addLine("Color Values retrieved. Proceeding to count pixels...");
        //int pix = frameMap.getPixel(320, 240);
        //telemetry.addLine("Pixel RGB: " + red(pix) + " / " + blue(pix) + " / " + green(pix));
        telemetry.addLine("Pixels counted: " + pixelCount);
        telemetry.update();
        //sleep(10000);
        return pixelCount;
    }

    public Bitmap getBarcodeBitmap() {
        initializeFrameQueue(2);
        //AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        Bitmap bmp = null;

        //telemetry.addLine("Attempting to Open Camera...");
        if (camera == null) return null;
        //telemetry.addLine("Camera Opened. Attempting to Start Camera...");
        if (cameraCaptureSession == null) return null;
        telemetry.addLine("Camera Started. Attempting to pull bmp from poll...");
        telemetry.update();
        while(true) {
            bmp = frameQueue.poll();
            if (bmp != null) {
                //onNewFrame(bmp);
                telemetry.addLine("bitmap pulled from camera");
                break;
            }
        }
        telemetry.update();
        return bmp;
    }

    private void onNewFrame(Bitmap frame) {
        //saveBitmap(frame);
        //frame.recycle(); // not strictly necessary, but helpful
    }

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
}
/*
___________________________________________________________________________________________________________________________________
-
-ODOMETRY GRAVEYARD
-
___________________________________________________________________________________________________________________________________
 */

//OdometryGlobalCoordinatePosition globalPositionUpdate;
//final double ODOMETRY_COUNTS_PER_INCH = 307.699557;

/*public void initOdometry(){
        //Initialize hardware map values.
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, ODOMETRY_COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }
     */
//THIS WOULD GO IN INITITIALIZE DRIVEBASE HARDWARE MAP
/*verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         */
/*
___________________________________________________________________________________________________________________________________
-
-NOTES!
-
___________________________________________________________________________________________________________________________________
 */
/*
left front motor = 0
left back motor = 1
right back motor = 2
right front motor = 3
 */
/*
    Motors:
    lfD
    lbD
    rbD
    rfD
    Intake
    pastaM
    shootM
    wobbleG

    Servos:
    wobbleS
    pastaS
    pastaS2

     String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
     */
