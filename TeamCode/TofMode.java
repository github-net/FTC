package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 working version
 no more grip
 */

@TeleOp(name="JopMode", group="Opmode")
//@Disabled
public class TofMode extends OpMode  {
    // Declare motors and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor back_leftDrive = null;
    private DcMotor back_rightDrive = null;
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    private CRServo capturing = null;
    //private Servo grip = null;
    private DcMotor lift = null;
    private DcMotor extension = null;

    private DistanceSensor sensorRange;
    private Rev2mDistanceSensor sensorTimeOfFlight;

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
        arm        = hardwareMap.get(DcMotor.class, "arm");
        arm2       = hardwareMap.get(DcMotor.class, "arm2");
        capturing = hardwareMap.get(CRServo.class, "capturing");
        lift = hardwareMap.get(DcMotor.class, "lift");
        //grip = hardwareMap.get(Servo.class, "grip");
        extension = hardwareMap.get(DcMotor.class, "extension");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        sensorRange = hardwareMap.get(DistanceSensor.class, "TOFsensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


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

        // generic DistanceSensor methods.
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format(Locale.US, "%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format(Locale.US, "%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format(Locale.US, "%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format(Locale.US, "%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods.
        telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        telemetry.update();


        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double backleftPower;
        double backrightPower;
        double leftpercentPower;
        double rightpercentPower;
        double backleftpercentPower;
        double backrightpercentPower;
        boolean button_up = gamepad2.a;             //a and b for arm
        boolean button_down = gamepad2.b;
        boolean button_press = gamepad2.x;          //x is lift up
        boolean reverse_button_press = gamepad2.y;  //y is lift down
        //boolean button_grip_close = gamepad2.left_bumper; //grip open
        //boolean button_grip_open = gamepad2.right_bumper; //grip close

        float arm_slow = gamepad2.right_trigger; //slows down arm
        float extension_slow = gamepad2.left_trigger; //slows down arm extension
        boolean arm_preset_up = gamepad2.dpad_up; //press dpad up to do arm preset up
        boolean arm_preset_down = gamepad2.dpad_down; //press dpad down to do arm preset down
        double armPower = gamepad2.left_stick_y; //arm extender
        float capturing_power = gamepad2.left_trigger; //left trigger for capturing power
        float reverse_capturing = gamepad2.right_trigger; //right trigger to reverse capturing
        double extensionPower = gamepad2.right_stick_y;

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
