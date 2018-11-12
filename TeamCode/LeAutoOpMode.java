package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Autonomous(name = "Autonomous Op Mode", group = "Servo")
@Disabled
/**
I tried
 -Justin
 */
public class LeAutoOpMode extends LinearOpMode {
    // Define class members
    DcMotor leftDrive;
    DcMotor rightDrive;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    private CRServo capturing = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //DcMotor motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //DigitalChannel digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //DistanceSensor sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //Servo servoTest = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm = hardwareMap.get(DcMotor.class,"arm");
        capturing = hardwareMap.get(CRServo.class,"capturing");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Wait for the start button
        telemetry.addData(">", "Press Start." );
        telemetry.update();
        waitForStart();
        //start op mode

        while (opModeIsActive()) {
            double leftPower;
            double rightPower;
            capturing.setPower(200);

        }
    }

}
