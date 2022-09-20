package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
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
    static final double STRAFE_SPEED = 0.4;
    static final double TARGET_SHOOTER_SPEED = 1.975;
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
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
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
        initUtilHardwareMap(util1name, util2name, util3name, util4name);
        initServoHardwareMap(servo1name, crservo1name, crservo2name);
        //IMU Stuff, sets up parameters and reports accelerations to logcat log
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmodeaz
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Used in Color Alignment
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        colorSensorLeft.enableLed(true);
        colorSensorRight.enableLed(true);
    }

    public void assignDriveBase(DcMotor rightfrontmotor, DcMotor rightbackmotor, DcMotor leftfrontmotor, DcMotor leftbackmotor) {
        rfmotor = rightfrontmotor;
        rbmotor = rightbackmotor;
        lfmotor = leftfrontmotor;
        lbmotor = leftbackmotor;
    }

    public void assignUtilMotors(DcMotor util1, DcMotor util2, DcMotor util3, DcMotor util4) {
        utilmotor1 = util1;
        utilmotor2 = util2;
        utilmotor3 = util3;
        utilmotor4 = util4;
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName) {
        rfmotor = hardwareMap.dcMotor.get(rfName);
        rfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor = hardwareMap.dcMotor.get(rbName);
        rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfmotor = hardwareMap.dcMotor.get(lfName);
        lfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbmotor = hardwareMap.dcMotor.get(lbName);
        lbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfmotor.setDirection(DcMotor.Direction.FORWARD);
        rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbmotor.setDirection(DcMotor.Direction.FORWARD);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfmotor.setDirection(DcMotor.Direction.REVERSE);
        lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbmotor.setDirection(DcMotor.Direction.REVERSE);
        lbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initUtilHardwareMap(String util1name, String util2name, String util3name, String util4name) {
        utilmotor1 = hardwareMap.dcMotor.get(util1name);
        utilmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor2 = hardwareMap.dcMotor.get(util2name);
        utilmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor3 = hardwareMap.dcMotor.get(util3name);
        utilmotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor4 = hardwareMap.dcMotor.get(util4name);
        utilmotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        utilmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        utilmotor1.setDirection(DcMotor.Direction.FORWARD);
        utilmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        utilmotor2.setDirection(DcMotor.Direction.FORWARD);
        utilmotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        utilmotor3.setDirection(DcMotor.Direction.FORWARD);
        utilmotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        utilmotor4.setDirection(DcMotor.Direction.FORWARD);
        utilmotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //THIS ONE IS YEAR SPECIFIC. WE MAY HAVE MORE OR LESS SERVOS AND CONTINUOUS ROTATION SERVOS THAN THIS
    private void initServoHardwareMap(String servo1name, String crservo1name, String crservo2name) {
        servo1 = hardwareMap.servo.get(servo1name);
        servo1.setPosition(0);
        crservo1 = hardwareMap.crservo.get(crservo1name);
        crservo1.setDirection(CRServo.Direction.FORWARD);
        crservo1.setPower(0);
        crservo2 = hardwareMap.crservo.get(crservo2name);
        crservo2.setDirection(CRServo.Direction.FORWARD);
        crservo2.setPower(0);
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
                leftSpeed = speed * -1;
            } else {
                leftSpeed = speed;
            }
            if (rightInches < 0) {
                rightSpeed = speed * -1;
            } else {
                rightSpeed = speed;
            }
            resetEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget = (lbmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
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
            rbmotor.setPower(0.7 * (rightSpeed + PI(desiredHeading)));
            rfmotor.setPower(0.7 * (rightSpeed + PI(desiredHeading)));
            lfmotor.setPower(0.7 * (leftSpeed - PI(desiredHeading)));
            lbmotor.setPower(0.7 * (leftSpeed - PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (rbmotor.isBusy()) && (rfmotor.isBusy()) && (lbmotor.isBusy()) && (lfmotor.isBusy())) {
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
                leftSpeed = (accelerate(lbmotor, leftSpeed, leftBackTarget) + accelerate(lfmotor, leftSpeed, leftFrontTarget) / 2);
                rightSpeed = (accelerate(rbmotor, rightSpeed, rightBackTarget) + accelerate(rfmotor, rightSpeed, rightFrontTarget) / 2);
                rbmotor.setPower((rightSpeed + PI(desiredHeading)));
                rfmotor.setPower((rightSpeed + PI(desiredHeading)));
                lfmotor.setPower((leftSpeed - PI(desiredHeading)));
                lbmotor.setPower((leftSpeed - PI(desiredHeading)));
            }

            lbmotor.setPower(0);
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            rbmotor.setPower(0);
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
                leftSpeed = speed * -1;
            } else {
                leftSpeed = speed;
            }
            if (rightInches < 0) {
                rightSpeed = speed * -1;
            } else {
                rightSpeed = speed;
            }
            resetEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget = (lbmotor.getCurrentPosition() - (int) (leftInches * ENCODER_COUNTS_PER_INCH));
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
            rbmotor.setPower(0.7 * (rightSpeed + PI(desiredHeading)));
            rfmotor.setPower(0.7 * (rightSpeed + PI(desiredHeading)));
            lfmotor.setPower(0.7 * (leftSpeed - PI(desiredHeading)));
            lbmotor.setPower(0.7 * (leftSpeed - PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (rbmotor.isBusy()) && (rfmotor.isBusy()) && (lbmotor.isBusy()) && (lfmotor.isBusy())) {
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
                telemetry.addData("rightSpeed", rightSpeed);
                telemetry.addData("leftSpeed", leftSpeed);
                telemetry.update();
                leftSpeed = (accelerate(lbmotor, leftSpeed, leftBackTarget) + accelerate(lfmotor, leftSpeed, leftFrontTarget) / 2);
                rightSpeed = (accelerate(rbmotor, rightSpeed, rightBackTarget) + accelerate(rfmotor, rightSpeed, rightFrontTarget) / 2);
                rbmotor.setPower((rightSpeed + PI(desiredHeading)));
                rfmotor.setPower((rightSpeed + PI(desiredHeading)));
                lfmotor.setPower((leftSpeed - PI(desiredHeading)));
                lbmotor.setPower((leftSpeed - PI(desiredHeading)));
            }

            lbmotor.setPower(0);
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            rbmotor.setPower(0);
            sleep(100);
        }
    }

    public void resetEncoders() {
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
    public void shoot(double time) {
        runtime.reset();
        utilmotor3.setPower(motor_power);
        sleep(900);
        while (runtime.seconds() < time) {
            utilmotor1.setPower(-1);
            utilmotor2.setPower(-1);
            utilmotor3.setPower(-1);
            crservo1.setPower(1);
            crservo2.setPower(-1);
        }
        utilmotor1.setPower(0);
        utilmotor2.setPower(0);
        utilmotor3.setPower(0);
        crservo1.setPower(0);
        crservo2.setPower(0);
    }

    public void smartShoot(double time) {
        double current_speed;
        double numberofRevolutions;
        double velocityoffset;
        runtime.reset();
        while (runtime.seconds() < time) {
            if (runtime.seconds() > 5.5) {
                utilmotor1.setPower(-1);
                utilmotor2.setPower(-1);
                crservo1.setPower(1);
                crservo2.setPower(-1);
            }
            numberofRevolutions = utilmotor3.getCurrentPosition() / COUNTS_PER_MOTOR_REV;
            current_speed = numberofRevolutions / runtime.seconds();
            velocityoffset = -TARGET_SHOOTER_SPEED - current_speed;
            velocityoffset = velocityoffset * 1.6;

            motor_power = -Math.abs(-1 + velocityoffset);
            utilmotor3.setPower(motor_power);
            telemetry.addData("Encoder Value", utilmotor3.getCurrentPosition());
            telemetry.addData("Current Speed", current_speed);
            telemetry.addData("Velocity offset", velocityoffset);
            telemetry.addData("Motor Power", motor_power);
            telemetry.update();

            //utilmotor3.setPower(-1);

        }
        utilmotor1.setPower(0);
        utilmotor2.setPower(0);
        utilmotor3.setPower(0);
        crservo1.setPower(0);
        crservo2.setPower(0);
    }

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
            rfmotor.setPower(1);
            rbmotor.setPower(1);
            lfmotor.setPower(1);
            lbmotor.setPower(1);
        }
        rfmotor.setPower(0);
        rbmotor.setPower(0);
        lfmotor.setPower(0);
        lbmotor.setPower(0);
    }

    public void strafeLeft(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rfmotor.setPower(1);
            rbmotor.setPower(-1);
            lfmotor.setPower(-1);
            lbmotor.setPower(1);
        }
        rfmotor.setPower(0);
        rbmotor.setPower(0);
        lfmotor.setPower(0);
        lbmotor.setPower(0);
    }

    public void strafeRight(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rfmotor.setPower(-1);
            rbmotor.setPower(1);
            lfmotor.setPower(1);
            lbmotor.setPower(-1);
        }
        rfmotor.setPower(0);
        rbmotor.setPower(0);
        lfmotor.setPower(0);
        lbmotor.setPower(0);
    }

    public void turnRight(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rfmotor.setPower(-0.5);
            rbmotor.setPower(-0.5);
            lfmotor.setPower(0.5);
            lbmotor.setPower(0.5);
            telemetry.addData("seconds or somethin", runtime.seconds());
            telemetry.update();
        }
        rfmotor.setPower(0);
        rbmotor.setPower(0);
        lfmotor.setPower(0);
        lbmotor.setPower(0);
    }

    public void turnLeft(double time) {
        runtime.reset();
        while (runtime.seconds() < time) {
            rfmotor.setPower(1);
            rbmotor.setPower(1);
            lfmotor.setPower(-1);
            lbmotor.setPower(-1);
        }
        rfmotor.setPower(0);
        rbmotor.setPower(0);
        lfmotor.setPower(0);
        lbmotor.setPower(0);
    }

    /*
    ___________________________________________________________________________________________________________________________________
    -
    -VISION METHODS!
    -
    ___________________________________________________________________________________________________________________________________
     */
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

        /*
    ___________________________________________________________________________________________________________________________________
    -
    -COLOR ALIGNMENT TEST PROGRAM
    -
    ___________________________________________________________________________________________________________________________________
     */

    private void driveByPower(double power) {
        lfmotor.setPower(power);
        lbmotor.setPower(power);
        rfmotor.setPower(power);
        rbmotor.setPower(power);
    }

    public void colorAlignment() {
        boolean notOnLine = true;
        lfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(notOnLine) {
            Color.RGBToHSV(colorSensorLeft.red() * 8, colorSensorLeft.green() * 8, colorSensorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesRight);

            //telemetry.addLine("HueLR: " + hsvValuesLeft[0] + ", " + hsvValuesRight[0]);
            //telemetry.addLine("SaturLR: " + hsvValuesLeft[1] + ", " + hsvValuesRight[1]);
            telemetry.addLine("ValLR: " + hsvValuesLeft[2] + ", " + hsvValuesRight[2]);

            if (hsvValuesLeft[2] >= 80 && hsvValuesRight[2] >= 80) {
                lfmotor.setPower(0);
                lbmotor.setPower(0);
                rfmotor.setPower(0);
                rbmotor.setPower(0);
                telemetry.addLine("Yay on the line");
                telemetry.update();
                notOnLine = false;
            }
            else if (hsvValuesLeft[2] >= 80) {
                telemetry.addLine("White line on LEFT Side");
                lfmotor.setPower(0);
                lbmotor.setPower(0);
                rfmotor.setPower(.3);
                rbmotor.setPower(.3);
            }
            else if (hsvValuesRight[2] >= 80) {
                telemetry.addLine("White line on RIGHT Side");
                lfmotor.setPower(.3);
                lbmotor.setPower(.3);
                rfmotor.setPower(0);
                rbmotor.setPower(0);
            }
            else {
                telemetry.addLine("No White line detected");
                lfmotor.setPower(-.2);
                lbmotor.setPower(-.2);
                rfmotor.setPower(-.2);
                rbmotor.setPower(-.2);
            }
        }
        lfmotor.setPower(0);
        lbmotor.setPower(0);
        rfmotor.setPower(.2);
        rbmotor.setPower(.2);
        sleep(500);
        lfmotor.setPower(0);
        lbmotor.setPower(0);
        rfmotor.setPower(0);
        rbmotor.setPower(0);

        lfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderDrive(DRIVE_SPEED, 1, 1, 10, 0);

        telemetry.update();
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
