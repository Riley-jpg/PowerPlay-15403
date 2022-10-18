package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="adultTeleop", group="Pushbot")
//@Disabled
public class old_Teleop extends LinearOpMode {

    public static final double UP_POSITION = .5;
    public static final double DOWN_POSITION = 1;
    public static final double OPEN_POSITION = .9;
    public static final double CLOSED_POSITION = .3;
    public static final double INLET_UP = 1;
    public static final double INLET_DOWN = 0.6;
    static final double SPIN = -1;

    baby_Hardware_Map robot = new baby_Hardware_Map();
    private ElapsedTime runtime = new ElapsedTime();

    private static int reverseDrive;

    public void runOpMode() {
        reverseDrive = 1;
        robot.init(hardwareMap);
        //robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boolean intakeSetUp = true;
        int armExtendUp = 1;
        //1 for down, 2 for up, 0 for manual
        int intakeDownEncoder = robot.intakemotor.getCurrentPosition();
        int intakeEncoderDifference = 0;

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //driving
            if (gamepad1.right_trigger > 0.1){
                if(gamepad1.left_trigger > 0.1) { //Reverse
                    robot.leftDrive.setPower(gamepad1.right_stick_y * -1 / 2);
                    robot.rightDrive.setPower(gamepad1.left_stick_y * -1 / 2);
                }
                else { //Forward
                    robot.leftDrive.setPower(gamepad1.left_stick_y / 2);
                    robot.rightDrive.setPower(gamepad1.right_stick_y / 2);
                }
            }
            else {
                if(gamepad1.left_trigger > 0.1) { //Reverse
                    robot.leftDrive.setPower(gamepad1.right_stick_y * -1);
                    robot.rightDrive.setPower(gamepad1.left_stick_y * -1);
                }
                else { //Forward
                    robot.leftDrive.setPower(gamepad1.left_stick_y);
                    robot.rightDrive.setPower(gamepad1.right_stick_y);
                }
            }

            //Extend and retract the Delivery Arm to any position
            /*if (gamepad2.a) {
                robot.armmotor.setPower(-1);    //Extend
            } else if (gamepad2.b) {
                robot.armmotor.setPower(1);     //Retract
            } else {
                robot.armmotor.setPower(0);     //Stop Moving (Brake)
            }*/
            if (gamepad2.right_stick_y > 0.2) {
                armExtendUp = 0;
                robot.armmotor.setPower(1);    //Extend
            }
            else if (gamepad2.right_stick_y < -0.2 && robot.armmotor.getCurrentPosition() > 0) {
                armExtendUp = 0;
                robot.armmotor.setPower(-1);     //Retract
            }
            else {
                if(armExtendUp == 0) {
                    armExtendUp = 0;
                    robot.armmotor.setPower(0);     //Stop Moving (Brake)
                }
            }
            telemetry.addData("Arm Extension", robot.armmotor.getCurrentPosition());

            //Arm auto extension to top
            if(gamepad2.b) {
                armExtendUp = 2;
            }
            else if(gamepad2.a) {
                armExtendUp = 1;
            }

            if(armExtendUp == 1) {
                robot.armservo.setPosition(UP_POSITION);
                if(robot.armmotor.getCurrentPosition() > 20) {
                    robot.armmotor.setPower(-1);
                }
                else {
                    robot.armmotor.setPower(0);
                }
            }
            else if(armExtendUp == 2){
                if(robot.armmotor.getCurrentPosition() < 3800) {
                    robot.armmotor.setPower(1);
                }
                else {
                    robot.armmotor.setPower(0);
                }
            }

            if(robot.armmotor.getCurrentPosition() > 200) {
                robot.inletdoor.setPosition(INLET_UP);
            }
            else{
                robot.inletdoor.setPosition(INLET_DOWN);
            }

            //Controls for the servo (Gate) that deposits a game element into a hub
            if (gamepad2.x){
                robot.armservo.setPosition(DOWN_POSITION);  //Drop gate
            }else if (gamepad2.y){
                robot.armservo.setPosition(UP_POSITION);    //Lift Gate

            }
            //Display encoder count on drive base motors for debugging
            telemetry.addData("left drive encoder", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right drive encoder", robot.rightDrive.getCurrentPosition());

            if(gamepad2.dpad_left) {
                robot.intakemotor.setPower(0.25);
                sleep(500);
                intakeDownEncoder = robot.intakemotor.getCurrentPosition();
                intakeSetUp = false;
            }

            intakeEncoderDifference = intakeDownEncoder - robot.intakemotor.getCurrentPosition();
            telemetry.addData("Intake Encoder Diff", intakeEncoderDifference);

            if(gamepad2.dpad_up) {          //If
                intakeSetUp = true;
            }
            else if(gamepad2.dpad_down) {
                intakeSetUp = false;
            }

            if(intakeSetUp == true) {
                if(intakeEncoderDifference < 380) {
                    robot.intakemotor.setPower(-0.7);
                }
                else if(intakeEncoderDifference < 420) {
                    robot.intakemotor.setPower(-0.1);
                }
                else {
                    robot.intakemotor.setPower(0);
                }
            }
            else {
                if(intakeEncoderDifference > 20) {
                    robot.intakemotor.setPower(0.25);
                }
                else {
                    robot.intakemotor.setPower(0);
                }
            }

            if (gamepad2.right_bumper){
                robot.spinnermotor.setPower(1);
            }else if (gamepad2.left_bumper){
                robot.spinnermotor.setPower(-1);
            }else {
                robot.spinnermotor.setPower(0);
            }

            if (gamepad1.b){
                robot.duckmotor.setPower(SPIN);
            }
            else if (gamepad1.x){
                robot.duckmotor.setPower(-SPIN);
            }
            else {
                robot.duckmotor.setPower(0);
            }

            telemetry.update();
        }


    }}
