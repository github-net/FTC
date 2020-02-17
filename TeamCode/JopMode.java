package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class JopMode extends OpMode  {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor back_leftDrive = null;
    private DcMotor back_rightDrive = null;
    private DcMotor intake_left = null;
    private DcMotor intake_right = null;
    private DcMotor lift_r = null;
    private DcMotor lift_l = null;
    private CRServo arm_r = null;
    private CRServo arm_l = null;
    private Servo grip = null;
    double motorPower=1.00;
    double gripPos = 0.85;
    @Override
    public void init() throws IllegalArgumentException {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        back_leftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        back_rightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intake_left = hardwareMap.get(DcMotor.class, "intake_left");
        intake_right = hardwareMap.get(DcMotor.class, "intake_right");
        lift_r = hardwareMap.get(DcMotor.class,"lift_r");
        lift_l = hardwareMap.get(DcMotor.class,"lift_l");
        arm_r = hardwareMap.get(CRServo.class,"arm_r");
        arm_l = hardwareMap.get(CRServo.class,"arm_l");
        grip = hardwareMap.get(Servo.class,"grip");
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        back_leftDrive.setDirection(DcMotor.Direction.FORWARD);
        back_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake_left.setDirection(DcMotor.Direction.REVERSE);
        intake_right.setDirection(DcMotor.Direction.FORWARD);
        lift_l.setDirection(DcMotor.Direction.FORWARD);
        lift_r.setDirection(DcMotor.Direction.REVERSE);
        grip.setDirection(Servo.Direction.FORWARD);
        telemetry.addData( "Status: ", "Successfully Initialized .-.");
    }
    @Override
    public void start() {
        runtime.reset();
    }
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
    public void loop() throws IllegalArgumentException {
        double leftPower;
        double rightPower;
        double backleftPower;
        double backrightPower;
        double leftpercentPower;
        double rightpercentPower;
        double backleftpercentPower;
        double backrightpercentPower;
        double drive  = gamepad1.left_stick_y; //up and down values
        double strafe =  -gamepad1.left_stick_x; //side to side values
        double rotate = gamepad1.right_stick_x;
        double intakePower = gamepad1.left_trigger;
        boolean motorPdown = gamepad1.dpad_down;
        boolean motorPup = gamepad1.dpad_up;

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
        lift_r.setPower(-gamepad2.left_stick_y*0.5);
        lift_l.setPower(-gamepad2.left_stick_y*0.5);
        if(gamepad2.left_stick_y==0){
            lift_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        arm_l.setPower(gamepad2.right_stick_y);
        arm_r.setPower(-gamepad2.right_stick_y);
        //grip
        if(gamepad2.x==true){
            if(gripPos==0.83){
                gripPos+=0.09;
                if(gripPos>0.92){
                    gripPos=0.92;
                }
            }
            else {
                gripPos-=0.09;
                if(gripPos<0.83){
                    gripPos=0.83;
                }

            }
        }
        grip.setPosition(gripPos);
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
        telemetry.addData("Grip Pos","(%.2f)", gripPos);
    }
}
