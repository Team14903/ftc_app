package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "TeleOp Basic Program")
public class BasicTeleOpProgram extends LinearOpMode {

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;

    //Declare Servos
    private Servo armServo;

    //Value positions for servos
    private static final double armRetractedPosition = 0.0;
    private static final double armExtendedPosition = 0.2;
    @Override
    public void runOpMode() throws InterruptedException{
        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        //Set motors to opposite directions(is reversable if needed)
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        armServo = hardwareMap.servo.get("motorArm");

        //Waits for person to press Play button
        waitForStart();



        while(opModeIsActive()){
            //Motor goes forward as joystick is pushed forward in a tank drive situation
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //Button for extended position
            if(gamepad1.a){
                armServo.setPosition(armExtendedPosition);
            }
            //Button for retracted position
            if(gamepad1.b){
                armServo.setPosition(armRetractedPosition);
            }
            idle();
        }
    }
}