package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
//@Disabled
@TeleOp(name= "TeleOp Basic Program")
public class RoverRuckusTeleOp extends LinearOpMode {

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLatching;

    //Declare Servos
    private Servo armServo;

    //Declare Sensors
    DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit


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
        latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        latchingTouchSensorUp = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorUp");
        latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);
        latchingTouchSensorDown.setMode(DigitalChannel.Mode.INPUT);


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
        while(latchingTouchSensorDown.getState()&& gamepad1.x){
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
            //If a button isn't pressed and upper limit isn't reached then ...
            //Set motor to linear slide power
            if(gamepad1.a && latchingTouchSensorUp.getState()){
                motorLatching.setPower(linearSlidePower);
            }
            //Button for retracted position
            //If a button isn't pressed and lower limit isn't reached then ...
            //Set motor to negative linear slide power
            if(gamepad1.b && latchingTouchSensorDown.getState()){
                motorLatching.setPower(-linearSlidePower);
            }
            if((latchingTouchSensorDown.getState() || latchingTouchSensorUp.getState())||(!gamepad1.a&&!gamepad1.b)){
                motorLatching.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLatching.setPower(0);
            }
            idle();
        }
    }
}