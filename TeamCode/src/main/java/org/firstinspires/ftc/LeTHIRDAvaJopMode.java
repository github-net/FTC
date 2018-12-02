package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

/**
 working version
 No grip for now
 */

@TeleOp(name="Le THIRD AvA jOp Mode", group="Iterative Opmode")
//@Disabled
public class LeTHIRDAvaJopMode extends OpMode
{
    // Declare motors and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor back_leftDrive = null;
    private DcMotor back_rightDrive = null;
    private DcMotor arm = null;
    private CRServo capturing = null;
    private Servo grip = null;
    private DcMotor lift = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Le Status", "is very Initialization");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        back_leftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        back_rightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        arm        = hardwareMap.get(DcMotor.class, "arm");
        capturing = hardwareMap.get(CRServo.class, "capturing");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grip = hardwareMap.get(Servo.class, "grip");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Le Status", "is very Initialization");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double backleftPower;
        double backrightPower;
        boolean button_up = gamepad2.a;             //a and b for arm
        boolean button_down = gamepad2.b;
        boolean button_press = gamepad2.x;          //x and y for lift
        boolean reverse_button_press = gamepad2.y;
        boolean button_grip_close = gamepad2.left_bumper; //grip close
        boolean button_grip_open = gamepad2.right_bumper; //grip open

        float capturing_power = gamepad2.left_trigger; //left trigger for capturing power
        double back_drive = gamepad1.right_stick_x; //back drive
        double back_turn = -gamepad1.right_stick_y; //back turn
        double drive =  -gamepad1.left_stick_y; //front drive
        double turn  =  gamepad1.left_stick_x; //front turn

        boolean arm_preset_up = gamepad2.dpad_up; //press dpad up to do arm preset up
        boolean arm_preset_down = gamepad2.dpad_down; //press dpad down to do arm preset down

        leftPower        = Range.clip(drive + turn, -0.75, 0.75) ;
        rightPower       = Range.clip(drive - turn, -0.75, 0.75) ;
        backleftPower    = Range.clip(back_drive + back_turn, -1.0, 1.0) ;
        backrightPower   = Range.clip(back_drive - back_turn, -1.0, 1.0) ;


        capturing.setPower(capturing_power * -200); //capturing power

        float capturingPower = (capturing_power * -200);

        int time;
        time = 250;
        /**
         * ARM PRESET UP
         */
        if(arm_preset_up == true){
            arm.setPower(-25);
            try{
                java.lang.Thread.sleep(550);
            }catch(InterruptedException ie){

            }
            arm.setPower(0);
        }
        /**
         * ARM PRESET DOWN
         */
        if(arm_preset_down == true){
            arm.setPower(25);
            try{
                java.lang.Thread.sleep(400);
            }catch(InterruptedException ie){

            }
            arm.setPower(-25);
            try{
                java.lang.Thread.sleep(50);
            }catch(InterruptedException ie){

            }
            arm.setPower(0);
        }
        /**
         * LIFT
         */
        if (button_press == true) {
            lift.setPower(1000);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            lift.setPower(0);
        }
        if (reverse_button_press == true) {
            lift.setPower(-1000);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            lift.setPower(0);
            /**
             * ARM
             */
        }
        if (button_up == true) {
            arm.setPower(150);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            arm.setPower(0);
        }
        if (button_down == true) {
            arm.setPower(-150);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            arm.setPower(0);
        }

        if (button_grip_open == true) {
            grip.setPosition(100); // Grip Open
        }
        if (button_grip_close == true) {
            grip.setPosition(0); // Grip Close
        }

        // Send calculated power to wheels

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        back_leftDrive.setPower(backleftPower);
        back_rightDrive.setPower(backrightPower);



        // Show the elapsed game time, wheel power, and capturing power
        telemetry.addData("Le StAtUs", "Le InitIaliZation TiMe: " + runtime.toString());
        telemetry.addData("Le MoToRs", "Left Motor Power: (%.2f) Right Motor Power (%.2f)", leftPower, rightPower);
        telemetry.addData("Le BaCk MoToRs", "Back Left Motor Power: (%.2f) Back Right Motor Power (%.2f)", backleftPower, backrightPower);
        telemetry.addData("Le Capturing", "Capturing Power: (%.2f)", capturingPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
