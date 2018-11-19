package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp
public class MySecondJavaOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private CRServo capturing = null;
    private DcMotor lift = null;


    DcMotor left;
    DcMotor right;

    @Override
    public void runOpMode() {
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
        lift = hardwareMap.get(DcMotor.class, "lift");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double leftPower;
            double rightPower;
            //change the negative
            boolean lift_button = gamepad2.x;
            boolean lift_button_reverse = gamepad2.y
            boolean button_press = gamepad2.a;
            boolean reverse_button_press = gamepad2.b;
            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -0.5, 0.5) ;
            rightPower = Range.clip(drive - turn, -0.5, 0.5) ;
            capturing.setPower(200);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            if (button_press == true) {
                arm.setPower(500);
                sleep(250);
                arm.setPower(0);
            }
            if (reverse_button_press == true) {
                arm.setPower(-500);
                sleep(250);
                arm.setPower(0);
            }
            if (lift_button == true) {
                lift.setPower(1500);
                sleep(250);
                arm.setPower(0);
            }if (lift_button_reverse == true) {
                lift.setPower(-1500);
                sleep(250);
                arm.setPower(0);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        // run until the end of the match (driver presses STOP)

        }
    }
}
