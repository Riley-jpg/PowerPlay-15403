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
import org.firstinspires.ftc.teamcode.babyHwMap;

@Autonomous(name="TensorFlowDriveTest", group="TensorFlowTest")
public class TensorFlowDriveTest extends Auto_Util {

    final double DESIRED_DISTANCE = 8.0;
    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AYSaE87/////AAABmd4MI42q9kBngeU2bY+LVZJDc/gsy7wMwK3XXPjV+w2c4E3gtwueNUhCeDOIXgW1qP0yVp+ZvTxaTspl7CXq3ogA6ZUIqqLep9WvnAF5xLF7KIZOITXPRKcAPeK3O6o7gazhB0RdNpZKTavtq2TAV/D9LME20zAAZcwoSVRGzmFmnhS3TDsaWMtC+kWQDLh+cOlqB/SoTzsg07av6GLyNlz2PxkZnVhPqMHaDjeYdOrgTzT8KqG8XtC9GtC7tuBbC8+bE8zBExb2ToydJ4BLFKhG38Tms8oCNoGYUs1j3h3reNN1Obx74RtWqGSxTVcvml0mB0XsnAChPKoGt7WFzWrNLwEZ1BJ2jDkzjNYaIfow";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";

}
