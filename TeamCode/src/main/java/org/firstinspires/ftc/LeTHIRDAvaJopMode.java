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
    no grip yet
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
    //private CRServo capturing = null;
    //private Servo grip = null;
    private DcMotor lift = null;
    private DcMotor extension = null;

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
        //capturing = hardwareMap.get(CRServo.class, "capturing");
        lift = hardwareMap.get(DcMotor.class, "lift");
        //grip = hardwareMap.get(Servo.class, "grip");
        extension = hardwareMap.get(DcMotor.class, "extension");

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
        boolean button_press = gamepad2.x;          //x is lift up
        boolean reverse_button_press = gamepad2.y;  //y is lift down
        //boolean button_grip_close = gamepad2.left_bumper; //grip open
        //boolean button_grip_open = gamepad2.right_bumper; //grip close
        float arm_slow = gamepad2.right_trigger; //slows down arm
        float extension_slow = gamepad2.left_trigger; //slows down arm extension
        boolean arm_preset_up = gamepad2.dpad_up; //press dpad up to do arm preset up
        boolean arm_preset_down = gamepad2.dpad_down; //press dpad down to do arm preset down
        double extensionPower = gamepad2.left_stick_y; //arm extender


        //float capturing_power = gamepad2.left_trigger; //left trigger for capturing power
        double drive  = -gamepad1.left_stick_y; //up and down values
        double strafe =  gamepad1.left_stick_x; //side to side values
        double rotate = -gamepad1.right_stick_x;
        double upright = gamepad1.right_trigger;
        double upleft = gamepad1.left_trigger;
        boolean downright = gamepad1.right_bumper;
        boolean downleft = gamepad1.left_bumper;

        leftPower        = Range.clip(drive + strafe + rotate, -0.5, 0.5) ;
        rightPower       = Range.clip(drive - strafe - rotate, -0.5, 0.5) ;
        backleftPower    = Range.clip(drive - strafe + rotate, -0.5, 0.5) ;
        backrightPower   = Range.clip(drive + strafe - rotate, -0.5, 0.5) ;

        extension.setPower(extensionPower/2);

        //stupid and inefficient if statements

        if(upright>0) {
            leftDrive.setPower(upright);
            back_rightDrive.setPower(-upright);
        }
        if(upleft>0) {
            rightDrive.setPower(upleft);
            back_leftDrive.setPower(upleft);
        }
        if(downright==true) {
            rightDrive.setPower(-1);
            back_leftDrive.setPower(-1);
        }
        if(downleft==true) {
            leftDrive.setPower(-1);
            back_rightDrive.setPower(1);
        }
        /**
        capturing.setPower(capturing_power * -200); //capturing power

        float capturingPower = (capturing_power * -200);
        */
        int time;
        time = 250;
        if(arm_slow > 0){
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(extension_slow > 0) {
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //ARM PRESET UP

        if(arm_preset_up == true){
            arm.setPower(-0.5);
            try{
                java.lang.Thread.sleep(550);
            }catch(InterruptedException ie){

            }
            arm.setPower(0);
        }
        //ARM PRESET DOWN

        if(arm_preset_down == true){
            arm.setPower(0.5);
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
        if (button_up == true) {
            arm.setPower(0.5);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            arm.setPower(0);
        }
        if (button_down == true) {
            arm.setPower(-0.5);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            arm.setPower(0);
        }
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
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        back_leftDrive.setPower(backleftPower);
        back_rightDrive.setPower(-backrightPower);

        // Show the elapsed game time, wheel power, and capturing power
        telemetry.addData("Le StAtUs", "Le InitIaliZation TiMe: " + runtime.toString());
        telemetry.addData("Le MoToRs", "Left Motor Power: (%.2f) Right Motor Power (%.2f)", leftPower, rightPower);
        telemetry.addData("Le BaCk MoToRs", "Back Left Motor Power: (%.2f) Back Right Motor Power (%.2f)", backleftPower, backrightPower);
        //telemetry.addData("Le Capturing", "Capturing Power: (%.2f)", capturingPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


}
//AQikMer/////AAABmQpFKno5UkD8ttfhJ5GRkmZdjeCp3jXXK0tXD4AOMCyQfQ7JufK5eQFX5/NKh0gL5PGgfU9y869kiL6AwKNX6SiGYkQasSnZSX4NehCseeQcxBkcwsksMBdCPSzK/m0zn3JNJWR3BP1cFOA5obzaP9UdUmN/ANV1T7tPSKkt2uf4qseDo3CcvA2JCW7zihI3DGg3Eq52wyziSMsFBM7NhZagUPy4SsdDd9KCOiPiD0VRE4ncmelAfAZQW0eOpid5izuLv9AS2Re2gKFAwOBtXydlfw1P0Pv4Q+guIBcgAofXFYRkNziJ6dqLF8XecT6Ocn4cykqnahj1OepN66iMhhdXMa4o9N/r8QezDTi5BEun
