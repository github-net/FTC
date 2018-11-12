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
 THIS IS THE ITERATIVE JAVA OP MODE
 working version i think
 */

@TeleOp(name="Le THIRD AvA jOp Mode", group="Iterative Opmode")
//@Disabled
public class LeTHIRDAvaJopMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private CRServo capturing = null;
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
        arm        = hardwareMap.get(DcMotor.class, "arm");
        capturing = hardwareMap.get(CRServo.class, "capturing");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Le Status", "is very Initialization");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        boolean button_up = gamepad1.a;
        boolean button_down = gamepad1.b;
        boolean button_press = gamepad1.x;
        boolean reverse_button_press = gamepad1.y;

        double drive =  -gamepad1.left_stick_y; //moving forward and backward
        double turn  =  gamepad1.left_stick_x; //turn left and right



        leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
        rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        int time;
        time = 250;
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
            arm.setPower(1000);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            arm.setPower(0);
        }
        if (button_down == true) {
            arm.setPower(-1000);
            //try and catch code
            try{
                java.lang.Thread.sleep(time);
            }catch(InterruptedException ie){

            }
            //end of try and catch code
            arm.setPower(0);
        }

        // Send calculated power to wheels
        capturing.setPower(200);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Le StAtUs", "Le InitIaliZation TiMe: " + runtime.toString());
        telemetry.addData("Le MoToRs", "Left Motor Power: (%.2f) Right Motor Power (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}