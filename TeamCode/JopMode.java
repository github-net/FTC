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
    private Servo grab_right = null;
    private Servo grab_left = null;
    double intakeCurrentPower=1.00;
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
        intake_left = hardwareMap.get(DcMotor.class, "intake_left");
        intake_right = hardwareMap.get(DcMotor.class, "intake_right");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grip = hardwareMap.get(Servo.class, "grip");
        fine = hardwareMap.get(Servo.class,"fine");
        rough = hardwareMap.get(Servo.class,"rough");
        grab_right = hardwareMap.get(Servo.class,"dad_right");
        grab_left = hardwareMap.get(Servo.class,"dad_left");

        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        back_leftDrive.setDirection(DcMotor.Direction.FORWARD);
        back_rightDrive.setDirection(DcMotor.Direction.FORWARD);
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
        double grab = gamepad2.left_trigger;
        double ungrab = gamepad2.right_trigger;
        double liftPower = gamepad2.left_stick_y;
        double rv = gamepad2.right_stick_x;


        boolean fineleft = gamepad2.left_bumper;
        boolean fineright = gamepad2.right_bumper;


        //gamepad 1
        double drive  = -gamepad1.left_stick_y; //up and down values
        double strafe =  -gamepad1.left_stick_x; //side to side values
        double rotate = -gamepad1.right_stick_x;
        double intakePower = gamepad1.left_trigger;
        boolean intakePdown = gamepad1.dpad_down;
        boolean intakePup = gamepad1.dpad_up;
        double intakeOut = gamepad1.right_trigger;
        
        //rough aka rv
        double roughPos = rough.getPosition();
        rough.setPosition(roughPos+=(rv*10));
        
        //fine servo
        double finePos = fine.getPosition();
        

        //intake power
        if(intakePup){
            intakeCurrentPower+=0.25;
        }
        if(intakePdown){
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


        //grip
        if(grab>0.5){
            grip.setPosition(0);
        }
        else if(ungrab>0.5){
            grip.setPosition(90);
        }

        //the everything
        if(everything){
            lift.setPower(-1);
            try { Thread.sleep(1000); } catch (InterruptedException e) { }
            rough.setPosition(90);
            lift.setPower(0);
        }
        
        //lift
        lift.setPower(liftPower);

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
        telemetry.addData("Le Intake Power", "(%.2f)", intakePower);
        telemetry.addData("Max Intake Power", "(%.2f)", intakeCurrentPower);
        telemetry.addData("Rough Pos", " (%.2f)", roughPos);
        telemetry.addData("Fine Pos","(.2f)", finePos);

        //sensor range stuff
        //telemetry.addData("deviceName",sensorRange.getDeviceName() );
        //telemetry.addData("range", String.format(Locale.US, "%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        // Rev2mDistanceSensor specific methods.
        //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

    }



}
