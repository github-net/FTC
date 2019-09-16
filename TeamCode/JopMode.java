package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    private CRServo capturing = null;
    //private Servo grip = null;
    private DcMotor lift = null;
    private DcMotor extension = null;

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

        /*
        arm.setPower(armPower);
        arm2.setPower(-armPower);
        extension.setPower(-extensionPower/1.5);


        //stupid and inefficient if statements

        //two wheel drive
        if(leftmotor>0) {
            leftDrive.setPower(-leftmotor);
            back_leftDrive.setPower(-leftmotor);
            telemetry.addData("2 Wheel Drive Activated","Left Wheels");
        }
        if(rightmotor>0) {
            rightDrive.setPower(-rightmotor);
            back_rightDrive.setPower(rightmotor);
            telemetry.addData("2 Wheel Drive Activated","Right Wheels");
        }
        if(reverse_leftmotor==true) {
            leftDrive.setPower(1);
            back_leftDrive.setPower(1);
            telemetry.addData("Reverse","Left Wheels");
        }
        if(reverse_rightmotor==true) {
            rightDrive.setPower(1);
            back_rightDrive.setPower(-1);
            telemetry.addData("Reverse","Right Wheels");
        }


        capturing.setPower(reverse_capturing-capturing_power); //capturing power


        float capturingPower = (-capturing_power + reverse_capturing);

        int time;
        time = 1;
        if(arm_slow > 0){
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(extension_slow > 0) {
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //ARM PRESET UP

        if(arm_preset_up == true){
            telemetry.addData("Eddie"," is a thot");
        }
        //ARM PRESET DOWN

        if(arm_preset_down == true){
            telemetry.addData("Gained"," Down Syndrome!");
        }
        //LIFT

        if (button_press == true) {
            lift.setPower(1);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            lift.setPower(0);
        }
        if (reverse_button_press == true) {
            lift.setPower(-1);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            lift.setPower(0);
            // ARM

        }
        */


        /**
         if (button_up == true) {
         extension.setPower(1);

         //try and catch code
         try{
         java.lang.Thread.sleep(time);
         }catch(InterruptedException ie){

         }
         //end of try and catch code
         extension.setPower(0);
         }
         if (button_down == true) {
         extension.setPower(-1);

         //try and catch code
         try{
         java.lang.Thread.sleep(time);
         }catch(InterruptedException ie){

         }
         //end of try and catch code
         extension.setPower(0);

         }
         */
        /**
         * grip

         if (button_grip_open == true) {
         grip.setPosition(100); // Grip close right bumper

         }
         if (button_grip_close == true) {
         grip.setPosition(0); // Grip open
         }
         */

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
