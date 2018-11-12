//https://firstinmaryland.org/images/docs/2016Docs/FTC_Kickoff/intro_to_autonomous_programming.pdf
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TestOp extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    Servo servoTest ;
    Servo leftClaw; //not in use
    Servo rightClaw; //not in use
    @Override
    public void runOpMode() throws InterruptedException{
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        //aryaman is bad at life
        servoTest = hardwareMap.servo.get("servoTest");

        waitForStart();

        leftDrive.setPower(1.0);
        rightDrive.setPower(-1.0);

        ElapsedTime eTime = new ElapsedTime();

        eTime.reset();

        while(eTime.time() < 2.5) {}

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        eTime.reset();

        while(eTime.time() < 2.5) {}



    }
}
