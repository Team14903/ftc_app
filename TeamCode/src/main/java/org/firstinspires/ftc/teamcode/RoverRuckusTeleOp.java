package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name= "TeleOp Basic Program")
public class BasicTeleOpProgram extends LinearOpMode {

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLatching;

    //Declare Servos
    private Servo armServo;

    //Declare Sensors
    TouchSensor latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    TouchSensor latchingTouchSensorUp; //Sensor to test if motor has reached upper limit


    //Value positions for servos
    private static final double armRetractedPosition = 0.0;
    private static final double armExtendedPosition = 0.2;

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static double linearSlidePower = 50; //Power for linear slide motors
    private static double turningPower; //Power difference between drive motors

    @Override
    public void runOpMode() throws InterruptedException{

        //Configure Sensors
        latchingTouchSensorDown = hardwareMap.touchSensor.get("latchingTouchSensorDown");
        latchingTouchSensorUp = hardwareMap.touchSensor.get("latchingTouchSensorUp");

        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLatching = hardwareMap.dcMotor.get("motorLatching");

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLatching.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        armServo = hardwareMap.servo.get("motorArm");

        //To set position of linear slide to down
        //While the bottom touch sensor is not pressed and x on gamepad1 is pressed ...
        //set the latching motor to 1/5 of normal power
        while(!latchingTouchSensorDown.isPressed()&& gamepad1.x){
            motorLatching.setPower(-linearSlidePower);
        }

        //Waits for person to press Play button
        waitForStart();


        while(opModeIsActive()){
            //y value is set to negative in the stick control to make up mean positive
            //Motor goes forward as joystick is pushed forward in a tank drive situation
            //motorLeft.setPower(-gamepad1.left_stick_y);
            //motorRight.setPower(-gamepad1.right_stick_y);
            // Motor goes forward with joystick and turns with same joystick
            driveMotorPower = -gamepad1.right_stick_y;
            turningPower = gamepad1.right_stick_x;
            motorLeft.setPower((driveMotorPower+turningPower)/2);
            motorRight.setPower((-driveMotorPower-turningPower)/2);

            //Button for extended position
            //If a button is pressed and upper limit isn't reached then ...
            //Set motor to linear slide power
            if(gamepad1.a && !latchingTouchSensorUp.isPressed()){
                motorLatching.setPower(linearSlidePower);
            }
            //Button for retracted position
            //If a button is pressed and lower limit isn't reached then ...
            //Set motor to negative linear slide power
            if(gamepad1.b && !latchingTouchSensorDown.isPressed()){
                motorLatching.setPower(-linearSlidePower);
            }
            
            idle();
        }
    }
}