package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 working version
 no more grip
 */

@TeleOp
//(name="JopMode", group="Opmode")
//@Disabled
public class Manual extends OpMode  {
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
    private Servo fine = null;
    private Servo rough = null;
    private Servo dad_right = null;
    private Servo dad_left = null;
    private Servo blockstick = null;
    double motorPower=1.00;
    //private DistanceSensor sensorRange;
    //private Rev2mDistanceSensor sensorTimeOfFlight;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double roughposition = 0.28;
    double fineposition= 0.94;
    double blockstickposition = 0.0;


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
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        back_leftDrive.setDirection(DcMotor.Direction.FORWARD);
        back_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake_left.setDirection(DcMotor.Direction.REVERSE);
        intake_right.setDirection((DcMotor.Direction.FORWARD));
        //sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        //initialization is complete.
        //killmenow
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

        double leftPower;
        double rightPower;
        double backleftPower;
        double backrightPower;
        double leftpercentPower;
        double rightpercentPower;
        double backleftpercentPower;
        double backrightpercentPower;
        //gamepad 1
        double drive  = gamepad1.left_stick_y; //up and down values
        double strafe =  -gamepad1.left_stick_x; //side to side values
        double rotate = gamepad1.right_stick_x;
        double intakePower = gamepad1.left_trigger;
        boolean motorPdown = gamepad1.dpad_down;
        boolean motorPup = gamepad1.dpad_up;
        //intake power
        if(motorPup==true&&(motorPower!=1)){
            motorPower+=0.25;
        }
        if(motorPdown==true&&(motorPower!=0)){
            motorPower-=0.25;
        }
        if(gamepad1.right_trigger>0){ //intake in
            intake_left.setPower(-gamepad1.right_trigger);
            intake_right.setPower(-gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger>0){ //intake out
            intake_left.setPower(gamepad1.left_trigger);
            intake_right.setPower(gamepad1.left_trigger);
        }
        else{
            intake_left.setPower(0);
            intake_right.setPower(0);
        }

        leftPower        = Range.clip(drive + strafe - rotate, -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        leftpercentPower = Range.clip(drive + strafe - rotate, -1.0, 1.0);
        rightPower       = Range.clip(drive - strafe + rotate,  -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        rightpercentPower = Range.clip(drive - strafe + rotate, -1.0, 1.0);
        backleftPower    = Range.clip(drive - strafe - rotate,  -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        backleftpercentPower = Range.clip(drive - strafe - rotate,  -1.0, 1.0) ;
        backrightPower   = Range.clip(drive + strafe + rotate,  -1.5+(getBatteryVoltage()/13), 1.5-(getBatteryVoltage()/13)) ;
        backrightpercentPower =  Range.clip(drive + strafe + rotate,  -1.0, 1.0) ;
        leftDrive.setPower(-leftPower*motorPower);
        rightDrive.setPower(-rightPower*motorPower);
        back_leftDrive.setPower(-backleftPower*motorPower);
        back_rightDrive.setPower(backrightPower*motorPower);
        telemetry.addData("Initialization time", ":" + runtime.toString());
        telemetry.addData("Front Motors", "Left Motor Power: (%.2f)  Right Motor Power (%.2f)", leftpercentPower*100, rightpercentPower*100);
        telemetry.addData("Back Motors", "Back Left Motor Power: (%.2f) Back Right Motor Power (%.2f)", backleftpercentPower*100, backrightpercentPower*100);
        telemetry.addData("Le Voltage", "(%.2f) V", getBatteryVoltage());
        telemetry.addData("Le Intake Power", "(%.2f)", intakePower);
        telemetry.addData("Max Motor Power", "(%.2f)", motorPower);
        telemetry.addData("Rough Pos", " (%.2f)", roughposition);
        telemetry.addData("Fine Pos","(%.2f)", fineposition);
        telemetry.addData("Block Stick Position"," (%.2f)", blockstickposition);

        //sensor range stuff
        //telemetry.addData("deviceName",sensorRange.getDeviceName() );
        //telemetry.addData("range", String.format(Locale.US, "%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        //Rev2mDistanceSensor specific methods.
        //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
    }
}
