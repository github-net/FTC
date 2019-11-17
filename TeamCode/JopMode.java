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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
    private Servo fine = null;
    private Servo rough = null;
    private Servo dad_right = null;
    private Servo dad_left = null;
    private Servo blockstick = null;
    double intakeCurrentPower=1.00;
    private DistanceSensor sensorRange;
    private Rev2mDistanceSensor sensorTimeOfFlight;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  roughposition = 0.28;
    double fineposition= 0.37;
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
        lift = hardwareMap.get(DcMotor.class, "lift");
        grip = hardwareMap.get(Servo.class, "grip");
        fine = hardwareMap.get(Servo.class,"fine");
        rough = hardwareMap.get(Servo.class,"rough");
        dad_right = hardwareMap.get(Servo.class,"dad_right");
        dad_left = hardwareMap.get(Servo.class,"dad_left");
        blockstick = hardwareMap.get(Servo.class, "blockstick");

        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        back_leftDrive.setDirection(DcMotor.Direction.FORWARD);
        back_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        fine.setDirection(Servo.Direction.FORWARD);
        rough.setDirection(Servo.Direction.FORWARD);
        grip.setDirection(Servo.Direction.REVERSE);
        blockstick.setDirection(Servo.Direction.FORWARD);
        dad_left.setDirection(Servo.Direction.FORWARD);
        dad_right.setDirection(Servo.Direction.FORWARD);

        //intake_left.setDirection(DcMotor.Direction.FORWARD);
        //intake_right.setDirection((DcMotor.Direction.REVERSE));

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

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double backleftPower;
        double backrightPower;
        double leftpercentPower;
        double rightpercentPower;
        double backleftpercentPower;
        double backrightpercentPower;


        //gamepad 2
        boolean everything = gamepad2.a;
        boolean rveverything = gamepad2.b;
        boolean Grip = gamepad2.x;
        double liftPower = gamepad2.left_stick_y;
        double rv = gamepad2.right_stick_x;
        boolean fineleft = gamepad2.dpad_left;
        boolean fineright = gamepad2.dpad_right;

        //gamepad 1
        double drive  = -gamepad1.left_stick_y; //up and down values
        double strafe =  -gamepad1.left_stick_x; //side to side values
        double rotate = -gamepad1.right_stick_x;
        double intakePower = gamepad1.left_trigger;
        boolean intakePdown = gamepad1.dpad_down;
        boolean intakePup = gamepad1.dpad_up;
        double intakeOut = gamepad1.right_trigger;
        boolean blockstickdown = gamepad1.a;

        //temp dadder (right =0.0down 0.3up left=0.3down 0.0up)
        double leftgrabPos = dad_left.getPosition();
        double rightgrabPos = dad_right.getPosition();
        boolean grableft = gamepad1.left_bumper;
        boolean grabright = gamepad1.right_bumper;
        if(grableft){
            if(leftgrabPos==0.3){//left down
                dad_left.setPosition(0);
            }
            if(leftgrabPos==0){//left up
                dad_left.setPosition(0.3);
            }
        }
        if(grabright){
            if(rightgrabPos==0.3){//right up?
                dad_right.setPosition(0);
            }
            if(rightgrabPos==0){
                dad_right.setPosition(0.3);
            }
        }

        //blockstick
        if(blockstickdown){
            blockstick.setPosition(1);
        }
        else {
            blockstick.setPosition(0.0);
        }
        blockstickposition=blockstick.getPosition();
        //rough aka rv
        if (rv>0) {
            // Keep stepping up until we hit the max value.
            roughposition += 0.72 ; //quarter: 18 third: 24 half: 36
            if (roughposition >= 1 ) {
                roughposition = MAX_POS;

            }
        }
        else if(rv<0){

            roughposition -= 0.72 ; //quarter: 18 third: 24 half: 36
            if (roughposition <= 0.28 ) {
                roughposition = MIN_POS;
            }
        }
        else if (rv==0){

        }
        if(roughposition<0.28){
            roughposition=0.28;
        }
        rough.setPosition(roughposition);

        if (fineleft) {
            // Keep stepping up until we hit the max value.
            fineposition += INCREMENT ;
            if (fineposition >= MAX_POS ) {
                fineposition = MAX_POS;

            }
        }
        else if(fineright){
            // Keep stepping down until we hit the min value.
            fineposition -= INCREMENT ;
            if (fineposition <= MIN_POS ) {
                fineposition = MIN_POS;
            }

        }
        fine.setPosition(fineposition);


//i wanna commit parentheses genocide

        //intake power
        if(intakePup==true&&(intakeCurrentPower!=1)){
            intakeCurrentPower+=0.25;
        }
        if(intakePdown==true&&(intakeCurrentPower!=0)){
            intakeCurrentPower-=0.25;
        }


        //intake
        if(intakeOut>0){
            intake_right.setPower(-intakeOut * intakeCurrentPower);
            intake_left.setPower(intakeOut * intakeCurrentPower);
        }
        else {
            intake_right.setPower(intakePower * intakeCurrentPower);
            intake_left.setPower(-intakePower * intakeCurrentPower);
        }


        double gripposition = grip.getPosition();
        //grip
        if(Grip==true){
            if(gripposition==1){ //close
                grip.setPosition(0);
            }
            if(gripposition==0) { //open
                grip.setPosition(1);
            }
        }

        //the everything
        if(everything){
            //lift.setPower(1);
            try { Thread.sleep(500); } catch (InterruptedException e) { }
            lift.setPower(0);
        }

        //lift
        lift.setPower(liftPower*0.5);
        if(liftPower==0){
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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
        telemetry.addData("Initialization time", ":" + runtime.toString());
        telemetry.addData("Front Motors", "Left Motor Power: (%.2f)  Right Motor Power (%.2f)", leftpercentPower*100, rightpercentPower*100);
        telemetry.addData("Back Motors", "Back Left Motor Power: (%.2f) Back Right Motor Power (%.2f)", backleftpercentPower*100, backrightpercentPower*100);
        telemetry.addData("Le Voltage", "(%.2f) V", getBatteryVoltage());
        telemetry.addData("Le Intake Power", "(%.2f)", intakePower);
        telemetry.addData("Max Intake Power", "(%.2f)", intakeCurrentPower);
        telemetry.addData("Rough Pos", " (%.2f)", roughposition);
        telemetry.addData("Fine Pos","(%.2f)", fineposition);
        telemetry.addData("Block Stick Position","(%.2f)", blockstickposition);
        if(rightgrabPos==0){
            telemetry.addData("Right Grabber Position", " down");
        }
        if(rightgrabPos==0.3){
            telemetry.addData("Right Grabber Position", " up");
        }
        if(leftgrabPos==0){
            telemetry.addData("Left Grabber Position", " up");
        }
        if(leftgrabPos==0.3){
            telemetry.addData("Left Grabber Position", " down");
        }


        //sensor range stuff
        //telemetry.addData("deviceName",sensorRange.getDeviceName() );
        //telemetry.addData("range", String.format(Locale.US, "%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        // Rev2mDistanceSensor specific methods.
        //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

    }



}
