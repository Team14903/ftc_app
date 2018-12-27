package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;

//@Disabled
@TeleOp(name= "TeleOp Basic Program")
public class RoverRuckusTeleOp extends LinearOpMode {

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLatching;
    private DcMotor motorArmRotate;
    private DcMotor motorArmExtend;

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
    private static double xCordForCheese; //Cheesy x coordinates for game pad 1 right stick
    private static double yCordForCheese; //Cheesy y coordinates for game pad 1 right stick
    private static double theataAngleDegrees; // Angle in degrees for Cheesy control system

    @Override
    public void runOpMode() throws InterruptedException{

        //Configure Sensors
        latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        latchingTouchSensorUp = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorUp");
        latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);
        latchingTouchSensorDown.setMode(DigitalChannel.Mode.INPUT);


        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");//drive motor left
        motorRight = hardwareMap.dcMotor.get("motorRight");//drive motor right
        motorLatching = hardwareMap.dcMotor.get("motorLatching");//motor to latch and pull robot up
        motorArmRotate = hardwareMap.dcMotor.get("motorArmRotate");//motor to rotate the arm for game objects
        motorArmExtend = hardwareMap.dcMotor.get("motorArmExtend");//motor to extend the arm for game objects

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLatching.setDirection(DcMotor.Direction.FORWARD);
        motorArmRotate.setDirection(DcMotor.Direction.FORWARD);
        motorArmExtend.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        armServo = hardwareMap.servo.get("motorArm");

        //To set position of linear slide to down
        //While the bottom touch sensor is not pressed and x on gamepad1 is pressed ...
        //set the latching motor to 1/5 of normal power
        while(latchingTouchSensorDown.getState()&& gamepad1.x){
            motorLatching.setPower(-linearSlidePower);
        }

        int gameMode = 0;
        String[] listOfGameModes = {"1 Joystick Arcade", "2 Joystick Arcade", "Cheesy Control","Tank Drive"};
        boolean oldUpDPadValue = false;
        boolean oldDownDPadValue = false;

        while(!gamepad1.dpad_right){
            boolean newUpDPadValue = gamepad1.dpad_up;
            boolean newDownDPadValue = gamepad1.dpad_down;
            if(newUpDPadValue&&!oldUpDPadValue&&!(gameMode==3)){
                gameMode = gameMode+1;
            }else if(newDownDPadValue&&!oldDownDPadValue&&!(gameMode==0)){
                gameMode = gameMode-1;
            }
            oldDownDPadValue=newDownDPadValue;
            oldUpDPadValue=newUpDPadValue;
            telemetry.addData("Controller 1 Game Mode is: ",listOfGameModes[gameMode]);
            telemetry.update();
        }
        //Waits for person to press Play button
        waitForStart();


        while(opModeIsActive()) {

            if (gameMode == 0) {
                //Arcade style with one joystick(simple y is power,x is the differential of the x and y values)
                // Motor goes forward with joystick and turns with same joystick

                driveMotorPower = -gamepad1.right_stick_y;
                turningPower = gamepad1.right_stick_x;
                motorLeft.setPower(Range.clip((driveMotorPower - turningPower),-1,1));
                motorRight.setPower(Range.clip((driveMotorPower + turningPower),-1,1));

            }else if(gameMode == 1) {
                // Arcade style with 2 joysticks(simple s(simple y is power,x is the differential of
                // the x and y values)
                // Motor goes forward with joystick and turns with different joysticks
                driveMotorPower = -gamepad1.left_stick_y;
                turningPower = gamepad1.right_stick_x;
                motorLeft.setPower(Range.clip((driveMotorPower - turningPower),-1,1));
                motorRight.setPower(Range.clip((driveMotorPower + turningPower),-1,1));
            }/*else if(gameMode == 2) { //Cheesey drive system isn't working and not top priority for rover ruckus
                //Cheesy driving controls, has left y joystick as power
                //x and y coordinates are the current values of the right joystick
                driveMotorPower = -gamepad1.left_stick_y;
                xCordForCheese = gamepad1.right_stick_x;
                yCordForCheese = -gamepad1.right_stick_y;
                //if the x coordinate is positive ...
                if(xCordForCheese>=0){
                    //code inside Math.asin(): finds the absolute value of the x coordinate if it
                    // extends as a linear line to the unit circle as a positive x value
                    theataAngleDegrees=Math.asin(Math.abs(Math.sqrt(1+1/(yCordForCheese/xCordForCheese))))*180/Math.PI;
                }else {
                    //code inside Math.asin(): finds the absolute value of the x coordinate if it
                    // extends as a linear line to the unit circle as a negative x value
                    theataAngleDegrees=Math.asin(-Math.abs(Math.sqrt(1+1/(yCordForCheese/xCordForCheese))))*180/Math.PI;
                }
                //If the degrees are above 180(which means it went around the unit circle once already
                //reverse the algorithm because the input needs to be below 180
                if(theataAngleDegrees>180){
                    if(yCordForCheese>=0){
                        theataAngleDegrees=-Math.asin(Math.abs(Math.sqrt(1+1/(yCordForCheese/xCordForCheese))))*180/Math.PI;
                    }else {
                        theataAngleDegrees=-Math.asin(Math.abs(Math.sqrt(1+1/(yCordForCheese/xCordForCheese))))*180/Math.PI;
                    }
                }
                //Make the input between 1 and -1
                turningPower = theataAngleDegrees/180;
                if(xCordForCheese<0){ // the < can be changed if driver wants to switch the negative and positive values
                    //if the angle is in the opposite direction, negate the turning power
                    turningPower=-turningPower;
                }
                motorLeft.setPower(Range.clip((driveMotorPower + turningPower),-1,1));
                motorRight.setPower(Range.clip((driveMotorPower - turningPower),-1,1));


            }*/else if(gameMode == 3){
                //y value is set to negative in the stick control to make up mean positive
                //Motor goes forward as joystick is pushed forward in a tank drive situation
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);
            }

            //Button for extended position
            //If a button is pressed and upper limit isn't reached then ...
            //Set motor to linear slide power
            if(gamepad1.a && latchingTouchSensorUp.getState()){
                motorLatching.setPower(linearSlidePower);
            }
            //Button for retracted position
            //If a button is pressed and lower limit isn't reached then ...
            //Set motor to negative linear slide power
            if(gamepad1.b && latchingTouchSensorDown.getState()){
                motorLatching.setPower(-linearSlidePower);
            }
            if((latchingTouchSensorDown.getState() || latchingTouchSensorUp.getState())||(!gamepad1.a&&!gamepad1.b)){
                motorLatching.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLatching.setPower(0);
            }

            if(gamepad1.left_bumper) {
                armServo.setPosition(0);
            } else if(gamepad1.right_bumper){
                armServo.setPosition(1);
            }
            //controller for arm
            if(!(gamepad2.left_stick_y<0.05&&gamepad2.left_stick_y>-0.05)){
                motorArmRotate.setPower(Range.clip(gamepad2.left_stick_y,-1,1));
            }else {
                motorArmRotate.setPower(0);
            }
            idle();
        }
    }
}