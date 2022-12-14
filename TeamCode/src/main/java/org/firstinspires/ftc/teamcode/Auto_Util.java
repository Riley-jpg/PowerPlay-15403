package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Auto_Util", group="abstract")
@Disabled
public abstract class Auto_Util extends LinearOpMode {
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
    static final double STRAFE_SPEED = 0.1;
    static final double LIFT_SPEED = 0.7;
    static final double TARGET_SHOOTER_SPEED = 1.975;

    //Vision Colors
    /*
    int maxAll = getColorInt(255, 255, 255, 255);
    int minAll = getColorInt(255, 0, 0, 0);
    int maxRed = getColorInt(255, 255, 150, 150);
    int minRed = getColorInt(255, 150, 0, 0);
    int maxBlue = getColorInt(255, 150, 255, 255);
    int minBlue = getColorInt(255, 100, 100, 100);
    int maxCap = getColorInt(255, 92, 255, 228);
    int minCap = getColorInt(255, 25, 181, 155);

     */

    public static final int RED = 1;
    public static final int BLUE = 2;
    public static final int CAP = 3;

    //Drive motors
    DcMotor rightfrontDrive, rightbackDrive, leftfrontDrive, leftbackDrive;
    //Utility motors
    DcMotor slideMotor, slideMotor2, utilmotor3, utilmotor4;
    //odometry encoders
    DcMotor verticalLeft, verticalRight, horizontal;
    //servos
   // Servo servo1;
    CRServo intakeServo, crservo2;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    static double motor_power;
    //Hardware Map Names for drive motors and odometry wheels. This may need to be changed between years if the config changes
    String rfName = "rfD", rbName = "rbD", lfName = "lfD", lbName = "lbD";
    String util1name = "rightSlide", util2name = "leftSlide"; //, util3name = "shootM", util4name = "wobbleG";
    String /*servo1name = "wobbleS",*/ intakeServoname = "intake"/*, crservo2name = "pastaS2"*/;
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    //Variables for Camera
    /*
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static int stackSize;
    private static final String VUFORIA_KEY =
            "ASr8vlr/////AAABmQLvbOpFkkU9uYwJWNx5o2Antqe3VGKoedUKq3jObB/CKqlUQVEt/vJFkLrOinRFu+wKPJJx1LZe8vYwTUNhYX0/ygb2Oukz3sgnh3k0TMAWBL0gJXnlaw2JzGzwXMy7kL4K1EUdIoWKJgyMSDkWDeNa9JXMelIkU0mgPhQ1PpSqfDiFWcIpalRHVDMF+lR7wR67jJjt7sUWe3TPc2RoUZI9Ratv22wKzXGZTWUEHcvPIkJRyZjjXzzWper4e7gVhJBLEtZA/0U5Nqlasyl0A39AzatrIkCAa16P3J8Z0KKtza1YSKZRYc/Sz022CaSqCtgtG1jq5oK14I2JjQZIufdNLNc9uaXz3qN08jRaxujJ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //Color Sensors
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    float hsvValuesLeft[] = {0F, 0F, 0F};
    float hsvValuesRight[] = {0F, 0F, 0F};

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

     */
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
    public void initAuto() {
        initDriveHardwareMap(rfName, rbName, lfName, lbName);
        initUtilHardwareMap(util1name, util2name);
        initServoHardwareMap(intakeServoname);
        //IMU Stuff, sets up parameters and reports accelerations to logcat log
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmodeaz
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Used in Color Alignment
        /*
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        colorSensorLeft.enableLed(true);
        colorSensorRight.enableLed(true);

         */
    }

    public void assignDriveBase(DcMotor rightfrontmotor, DcMotor rightbackmotor, DcMotor leftfrontmotor, DcMotor leftbackmotor) {
        rightfrontDrive = rightfrontmotor;
        rightbackDrive = rightbackmotor;
        leftfrontDrive = leftfrontmotor;
        leftbackDrive = leftbackmotor;
    }

    public void assignUtilMotors(DcMotor slide1, DcMotor slide2, DcMotor util3, DcMotor util4) {
        slideMotor = slide1;
        slideMotor2 = slide2;
        utilmotor3 = util3;
        utilmotor4 = util4;
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName) {
        rightfrontDrive = hardwareMap.dcMotor.get(rfName);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackDrive = hardwareMap.dcMotor.get(rbName);
        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfrontDrive = hardwareMap.dcMotor.get(lfName);
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackDrive = hardwareMap.dcMotor.get(lbName);
        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initUtilHardwareMap(String slide1, String slide2) {
        slideMotor = hardwareMap.dcMotor.get(slide1);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2 = hardwareMap.dcMotor.get(slide2);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
        utilmotor3 = hardwareMap.dcMotor.get(util3name);
        utilmotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor4 = hardwareMap.dcMotor.get(util4name);
        utilmotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         */
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setDirection(DcMotor.Direction.FORWARD);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
        utilmotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        utilmotor3.setDirection(DcMotor.Direction.FORWARD);
        utilmotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        utilmotor4.setDirection(DcMotor.Direction.FORWARD);
        utilmotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */
    }

    //THIS ONE IS YEAR SPECIFIC. WE MAY HAVE MORE OR LESS SERVOS AND CONTINUOUS ROTATION SERVOS THAN THIS
    private void initServoHardwareMap(String crservo1name) {
        //servo1 = hardwareMap.servo.get(servo1name);
        //servo1.setPosition(0);
        intakeServo = hardwareMap.crservo.get(crservo1name);
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeServo.setPower(0);
        //crservo2 = hardwareMap.crservo.get(crservo2name);
        //crservo2.setDirection(CRServo.Direction.FORWARD);
        //crservo2.setPower(0);
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
        } else {
            return 0;
        }
    }

    public double accelerate(DcMotor motor, double speed, double target) {
        if (motor.getCurrentPosition() < (target / 10)) {
            return speed * 1.30;
        } else if (motor.getCurrentPosition() > (target * 8 / 10)) {
            return speed * 0.9;
        }
        return speed;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, double desiredHeading) {
        int leftBackTarget;
        int rightBackTarget;
        int rightFrontTarget;
        int leftFrontTarget;
        //int averageTarget;
        double leftSpeed, rightSpeed;
        if (opModeIsActive()) {
            if (leftInches < 0) {
                leftSpeed = speed * -0.1;
            } else {
                leftSpeed = speed;
            }
            if (rightInches < 0) {
                rightSpeed = speed * -0.1;
            } else {
                rightSpeed = speed;
            }
            resetMotorEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget = (leftbackDrive.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightBackTarget = (rightbackDrive.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            leftFrontTarget = (leftfrontDrive.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightFrontTarget = (rightfrontDrive.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            leftfrontDrive.setTargetPosition(leftFrontTarget);
            leftbackDrive.setTargetPosition(leftBackTarget);
            rightfrontDrive.setTargetPosition(rightFrontTarget);
            rightbackDrive.setTargetPosition(rightBackTarget);


            // Turn On RUN_TO_POSITION
            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightbackDrive.setPower(0.1 * (rightSpeed + PI(desiredHeading)));
            rightfrontDrive.setPower(0.1 * (rightSpeed + PI(desiredHeading)));
            leftfrontDrive.setPower(0.1 * (leftSpeed - PI(desiredHeading)));
            leftbackDrive.setPower(0.1 * (leftSpeed - PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (rightbackDrive.isBusy()) && (rightfrontDrive.isBusy()) && (leftbackDrive.isBusy()) && (leftfrontDrive.isBusy())) {
                telemetry.addData("Left Back Current Position", leftbackDrive.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rightbackDrive.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", leftfrontDrive.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rightfrontDrive.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                //telemetry.addData("rightSpeed",rightSpeed);
                //telemetry.addData("leftSpeed",leftSpeed);
                telemetry.addData("heading", heading(imu));
                telemetry.update();
                leftSpeed = (accelerate(leftbackDrive, leftSpeed, leftBackTarget) + accelerate(leftfrontDrive, leftSpeed, leftFrontTarget) / 2);
                rightSpeed = (accelerate(rightbackDrive, rightSpeed, rightBackTarget) + accelerate(rightfrontDrive, rightSpeed, rightFrontTarget) / 2);
                rightbackDrive.setPower((rightSpeed + PI(desiredHeading)));
                rightfrontDrive.setPower((rightSpeed + PI(desiredHeading)));
                leftfrontDrive.setPower((leftSpeed - PI(desiredHeading)));
                leftbackDrive.setPower((leftSpeed - PI(desiredHeading)));
            }

            rightfrontDrive.setPower(0);
            rightbackDrive.setPower(0);
            leftfrontDrive.setPower(0);
            leftbackDrive.setPower(0);
            sleep(100);
        }
    }

    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS, double desiredHeading) {
        int leftBackTarget;
        int rightBackTarget;
        int rightFrontTarget;
        int leftFrontTarget;
        //int averageTarget;
        double leftSpeed, rightSpeed;
        if (opModeIsActive()) {
            if (leftInches < 0) {
                leftSpeed = speed * -0.1;
            } else {
                leftSpeed = speed;
            }
            if (rightInches < 0) {
                rightSpeed = speed * -0.1;
            } else {
                rightSpeed = speed;
            }
            resetMotorEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget = (leftbackDrive.getCurrentPosition() - (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightBackTarget = (rightbackDrive.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            leftFrontTarget = (leftfrontDrive.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightFrontTarget = (rightfrontDrive.getCurrentPosition() - (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            leftfrontDrive.setTargetPosition(leftFrontTarget);
            leftbackDrive.setTargetPosition(leftBackTarget);
            rightfrontDrive.setTargetPosition(rightFrontTarget);
            rightbackDrive.setTargetPosition(rightBackTarget);


            // Turn On RUN_TO_POSITION
            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightbackDrive.setPower(0.1 * (rightSpeed + PI(desiredHeading)));
            rightfrontDrive.setPower(0.1 * (rightSpeed + PI(desiredHeading)));
            leftfrontDrive.setPower(0.1 * (leftSpeed - PI(desiredHeading)));
            leftbackDrive.setPower(0.1 * (leftSpeed - PI(desiredHeading)));



            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (rightbackDrive.isBusy()) && (rightfrontDrive.isBusy()) && (leftbackDrive.isBusy()) && (leftfrontDrive.isBusy())) {
                telemetry.addData("Left Back Current Position", leftbackDrive.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rightbackDrive.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", leftfrontDrive.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rightfrontDrive.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                telemetry.addData("heading", heading(imu));
                //telemetry.addData("Average Target",averageTarget);
                telemetry.addData("rightSpeed", rightSpeed);
                telemetry.addData("leftSpeed", leftSpeed);
                telemetry.update();
                leftSpeed = (accelerate(leftbackDrive, leftSpeed, leftBackTarget) + accelerate(leftfrontDrive, leftSpeed, leftFrontTarget) / 2);
                rightSpeed = (accelerate(rightbackDrive, rightSpeed, rightBackTarget) + accelerate(rightfrontDrive, rightSpeed, rightFrontTarget) / 2);
                rightbackDrive.setPower((rightSpeed + PI(desiredHeading)));
                rightfrontDrive.setPower((rightSpeed + PI(desiredHeading)));
                leftfrontDrive.setPower((leftSpeed - PI(desiredHeading)));
                leftbackDrive.setPower((leftSpeed - PI(desiredHeading)));
            }

            leftbackDrive.setPower(0);
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            rightbackDrive.setPower(0);
            sleep(100);
        }
    }

    public void encoderLift(double speed, double liftInches,  double timeoutS, double desiredHeading){
        int heightTarget;
        //int averageTarget;
        double liftSpeed;
        if (opModeIsActive()) {
            if (liftInches < 0) {
                liftSpeed = speed * -1;
            } else {
                liftSpeed = speed;
            }
            resetSlideEncoders();
            // Determine new target position, and pass to motor controller
            heightTarget = (slideMotor.getCurrentPosition() - (int) (liftInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            slideMotor.setTargetPosition(heightTarget);

            // Turn On RUN_TO_POSITION
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            slideMotor.setPower(0.7 * (liftSpeed + PI(desiredHeading)));
            slideMotor2.setPower(0.7 * (liftSpeed + PI(desiredHeading)));



            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (slideMotor.isBusy()) && (slideMotor2.isBusy())) {
                telemetry.addData("Lift Motor 1 Current Position", slideMotor.getCurrentPosition());
                telemetry.addData("Lift Motor 1 Desired Position", heightTarget);
                telemetry.addData("Lift Motor 2 Current Position", slideMotor2.getCurrentPosition());
                telemetry.addData("Lift Motor 2 Desired Position", heightTarget);
                telemetry.addData("heading", heading(imu));
                //telemetry.addData("Average Target",averageTarget);
                telemetry.addData("Lift 1 Speed", liftSpeed);
                telemetry.addData("Lift 2 Speed", liftSpeed);
                telemetry.update();
                liftSpeed = (accelerate(slideMotor, liftSpeed, heightTarget));
                slideMotor.setPower((liftSpeed + PI(desiredHeading)));
            }

            slideMotor.setPower(0);
            slideMotor2.setPower(0);
            sleep(100);
        }
    }

    public void resetMotorEncoders() {
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetSlideEncoders() {
        try {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (NullPointerException npe) {
            if (slideMotor == null) {
                telemetry.addData("SlideMotor is null", "Fix it");
                telemetry.update();
            }
        }
    }

    /*
    ___________________________________________________________________________________________________________________________________
    -
    -VISION METHODS!
    -
    ___________________________________________________________________________________________________________________________________
     */
    /*
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
    /*
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
        //final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        /*
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        /*
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
    /*
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                /*
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                /*
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
                    } catch (CameraException |RuntimeException e) {
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
    /*
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
    /*
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

    */
    /*
    ___________________________________________________________________________________________________________________________________
    -
    -GAME SPECIFIC METHODS! SCRAP AFTER THIS YEAR!
    -
    ___________________________________________________________________________________________________________________________________
     */

    /*
    ___________________________________________________________________________________________________________________________________
    -
    -BASIC (UNTESTED) DRIVE BY TIME METHODS. THESE SHOULD HELP WITH DEBUGGING ONCE THEY, Y'KNOW, GET DEBUGGED
    -
    ___________________________________________________________________________________________________________________________________
     */

    public void setAllDriveMotors(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
        }
        rightfrontDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
    }

    public void strafeLeft(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(-1);
            leftfrontDrive.setPower(-1);
            leftbackDrive.setPower(1);
        }
        rightfrontDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
    }

    public void strafeRight(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rightfrontDrive.setPower(-1);
            rightbackDrive.setPower(1);
            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(-1);
        }
        rightfrontDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
    }

    public void turnRight(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rightfrontDrive.setPower(-0.5);
            rightbackDrive.setPower(-0.5);
            leftfrontDrive.setPower(0.5);
            leftbackDrive.setPower(0.5);
            telemetry.addData("seconds or somethin", runtime.seconds());
            telemetry.update();
        }
        rightfrontDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
    }

    public void turnLeft(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
            leftfrontDrive.setPower(-1);
            leftbackDrive.setPower(-1);
        }
        rightfrontDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
    }


    public void slideLift(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            slideMotor.setPower(0.5);
            slideMotor2.setPower(-0.5);
        }
        slideMotor.setPower(0);
        slideMotor2.setPower(0);
    }

    public void slideLower(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            slideMotor.setPower(-0.5);
            slideMotor2.setPower(0.5);
        }
        slideMotor.setPower(0);
        slideMotor.setPower(0);
    }



    /*
    ___________________________________________________________________________________________________________________________________
    -
    -VISION METHODS!
    -
    ___________________________________________________________________________________________________________________________________
     */
    /*
    public int ub_vision() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                    stackSize = 2;
                    return stackSize;
                } else if (stackSize != 2 && recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                    stackSize = 1;
                }
                telemetry.addData("Recognition Label: ", recognition.getLabel());
            }
        }
        return stackSize;
    }

    public void initCamera() {
        initVuforia();
        telemetry.addData("completed vuforia init", "uhuh");
        initTfod();
        stackSize = 0;
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //SWITCH FOR MIGRATING BETWEEN SMARTPHONE AND WEBCAM
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

     */

        /*
    ___________________________________________________________________________________________________________________________________
    -
    -COLOR ALIGNMENT TEST PROGRAM
    -
    ___________________________________________________________________________________________________________________________________
     */
/*
    private void driveByPower(double power) {
        rightfrontDrive.setPower(power);
        rightbackDrive.setPower(power);
        leftfrontDrive.setPower(power);
        leftbackDrive.setPower(power);
    }

 */
    /*

    public void colorAlignment() {
        boolean notOnLine = true;
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(notOnLine) {
            Color.RGBToHSV(colorSensorLeft.red() * 8, colorSensorLeft.green() * 8, colorSensorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesRight);

            //telemetry.addLine("HueLR: " + hsvValuesLeft[0] + ", " + hsvValuesRight[0]);
            //telemetry.addLine("SaturLR: " + hsvValuesLeft[1] + ", " + hsvValuesRight[1]);
            telemetry.addLine("ValLR: " + hsvValuesLeft[2] + ", " + hsvValuesRight[2]);

            if (hsvValuesLeft[2] >= 80 && hsvValuesRight[2] >= 80) {
                rightfrontDrive.setPower(0);
                rightbackDrive.setPower(0);
                leftfrontDrive.setPower(0);
                leftbackDrive.setPower(0);
                telemetry.addLine("Yay on the line");
                telemetry.update();
                notOnLine = false;
            }
            else if (hsvValuesLeft[2] >= 80) {
                telemetry.addLine("White line on LEFT Side");
                leftfrontDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightfrontDrive.setPower(.3);
                rightbackDrive.setPower(.3);
            }
            else if (hsvValuesRight[2] >= 80) {
                telemetry.addLine("White line on RIGHT Side");
                leftfrontDrive.setPower(.3);
                leftbackDrive.setPower(.3);
                rightfrontDrive.setPower(0);
                rightbackDrive.setPower(0);
            }
            else {
                telemetry.addLine("No White line detected");
                leftfrontDrive.setPower(-.2);
                leftbackDrive.setPower(-.2);
                rightfrontDrive.setPower(-.2);
                rightbackDrive.setPower(-.2);
            }
        }
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
        rightfrontDrive.setPower(.2);
        rightbackDrive.setPower(.2);
        sleep(500);
        leftfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
        rightfrontDrive.setPower(0);
        rightbackDrive.setPower(0);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderDrive(DRIVE_SPEED, 1, 1, 10, 0);

        telemetry.update();
    }
    */
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
