package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class babyHwMap extends HardwareMapUtil {

    HardwareMap hwmap = null;

    /* Public OpMode members. */
    public DcMotor leftfrontDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor leftbackDrive = null;
    public DcMotor rightbackDrive = null;
    public DcMotor linearActuator = null;
    public DcMotor linearActuator2 = null;
    public DcMotor slideMotor = null;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftfrontDrive = HardwareInitMotor("lfD", true);
        rightfrontDrive = HardwareInitMotor ("rfD", false);
        leftbackDrive = HardwareInitMotor ("lbD", true);
        rightbackDrive = HardwareInitMotor ("rbD", false);
        slideMotor = HardwareInitMotor("slide", true);
        //linearActuator = HardwareInitMotor("linAct1", true);
        //linearActuator2 = HardwareInitMotor("linAct2", true);

    }
}