package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 working version
 no more grip
 */

@TeleOp(name="JopMode", group="Opmode")
//@Disabled
public class JopMode extends OpMode  {
    // Declare motors and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor back_leftDrive = null;
    private DcMotor back_rightDrive = null;
    private DcMotor intake_left = null;
    private DcMotor intake_right = null;
    private DcMotor lift = null;
    private Servo grip = null;
    private Servo rv = null;


    /*
    initialization
    pew pew justin is here
     */
    @Override
    public void init() throws IllegalArgumentException {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        back_leftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        back_rightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intake_left = hardwareMap.get(DcMotor.class, "intake_left");
        intake_right = hardwareMap.get(DcMotor.class, "intake_right");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grip = hardwareMap.get(Servo.class, "grip");
        rv = hardwareMap.get(Servo.class,"rv");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake_left.setDirection(DcMotor.Direction.FORWARD);
        intake_right.setDirection((DcMotor.Direction.REVERSE));



        // Tell the driver that initialization is complete.
        telemetry.addData( "Status: ", "Successfully Initialized .-.");
    }
    @Override
    public void start() {
        runtime.reset();
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    @Override
    public void loop() throws IllegalArgumentException
    {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double backleftPower;
        double backrightPower;
        double leftpercentPower;
        double rightpercentPower;
        double backleftpercentPower;
        double backrightpercentPower;
        boolean grip_close = gamepad2.a;
        boolean grip_open = gamepad2.b;
        boolean rv_left = gamepad2.x;
        boolean rv_right = gamepad2.y;
        float intake_in = gamepad2.right_trigger;
        float intake_out = gamepad2.left_trigger;
        double lift = gamepad2.left_stick_y;
        double rv = gamepad2.right_stick_x;


        boolean button_grip_close = gamepad2.left_bumper;
        boolean button_grip_open = gamepad2.right_bumper;
        boolean arm_preset_up = gamepad2.dpad_up;
        boolean arm_preset_down = gamepad2.dpad_down;


        double drive  = -gamepad1.left_stick_y; //up and down values
        double strafe =  gamepad1.left_stick_x; //side to side values
        double rotate = -gamepad1.right_stick_x;
        double rightmotor = gamepad1.right_trigger;
        double leftmotor = gamepad1.left_trigger;
        boolean reverse_rightmotor = gamepad1.right_bumper;
        boolean reverse_leftmotor = gamepad1.left_bumper;

        //POWER SETTING

        leftPower        = Range.clip(drive + strafe - rotate, -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        leftpercentPower = Range.clip(drive + strafe - rotate, -1.0, 1.0);

        rightPower       = Range.clip(drive - strafe + rotate,  -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        rightpercentPower = Range.clip(drive - strafe + rotate, -1.0, 1.0);

        backleftPower    = Range.clip(drive - strafe - rotate,  -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        backleftpercentPower = Range.clip(drive - strafe - rotate,  -1.0, 1.0) ;

        backrightPower   = Range.clip(drive + strafe + rotate,  -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        backrightpercentPower =  Range.clip(drive + strafe + rotate,  -1.0, 1.0) ;


        // Send calculated power to wheels
        leftDrive.setPower(-leftPower);
        rightDrive.setPower(-rightPower);
        back_leftDrive.setPower(-backleftPower);
        back_rightDrive.setPower(backrightPower);

        // Show the elapsed game time, wheel power, and capturing power
        telemetry.addData("Initialization time", " :" + runtime.toString());
        telemetry.addData("Front Motors", "Left Motor Power: (%.2f)  Right Motor Power (%.2f)", leftpercentPower*100, rightpercentPower*100);
        telemetry.addData("Back Motors", "Back Left Motor Power: (%.2f) Back Right Motor Power (%.2f)", backleftpercentPower*100, backrightpercentPower*100);
        telemetry.addData("Le Voltage", "(%.2f) V", getBatteryVoltage());
        //telemetry.addData("Le Capturing", "Capturing Power: (%.2f)", capturingPower);
    }



}
