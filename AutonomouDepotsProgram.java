/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous OpMode", group="Linear Opmode")
//@Disabled
public class AutonomouDepotsProgram extends LinearOpMode {
    VuforiaNavRoverRuckus Vuforia = new VuforiaNavRoverRuckus();
    ConceptTensorFlowObjectDetection Tensor = new ConceptTensorFlowObjectDetection();
    private String Picture;

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLatching;

    //Declare Servos
    private Servo armServo;


    //Declare Sensors
    DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static double linearSlidePower = 50; //Power for linear slide motors
    private String goldPosition= null;

    //Define Gyro
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        int timesOpModeRun = 12;
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
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLatching.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        armServo = hardwareMap.servo.get("motorArm");

        // Initialize motors
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit       = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit       = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled  =false;
        parameters.mode            =BNO055IMU.SensorMode.IMU;
        parameters.loggingTag      ="IMU";
        imu                        =hardwareMap.get(BNO055IMU.class, "imu name");
        imu.initialize(parameters);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            //while upper limit isn't reached(robot hasn't lowed itself)...
            //while (latchingTouchSensorUp.getState()){
                //motorLatching.setPower(linearSlidePower);
            //}
            motorLatching.setPower(0);
            //Turn robot toward depot
            TurnGyro(180,3,0.5);

            //Move toward Depot while knocking out center game piece(Sampling Mission)
            //Distance Total 1366 Counts or 52.8inches
            DriveFwdAccDcc(0, 1, 200);
            DriveFwdDistance(1, 966, false);
            DriveFwdAccDcc(1, 0, 200);
            //Place team marker
            armServo.setPosition(1);
            armServo.setPosition(0);
            //Turn robot towards the crater
            TurnGyro(-50,3,0.5);
            //Move until hits crater(sense reduction of speed)
            DriveFwdAccDcc(0,-1,200);
            DriveFwdObstacle(-1,-0.7);

            //A bunch of telemetry data for quick debugging
            telemetry.addData("Wheel Encoder", (motorLeft.getCurrentPosition()+motorRight.getCurrentPosition())/2);
            telemetry.addData("Gyro angle",imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES));
            telemetry.addData("Right Wheel Encoder", motorRight.getCurrentPosition());
            telemetry.addData("Left Wheel Encoder", motorLeft.getCurrentPosition());
            telemetry.update();
            Log.i("Vuforia",Picture);
            requestOpModeStop();

        }
    }
    // This is a method to make code easier to read, see above
    public void DriveFwdDistance(double Power,int distance,boolean stop) throws InterruptedException {
        // Get current average motor position
        // Core hex motor with 90mm wheel is 0.03865148441 inches per count
        int motorPosition = (-motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;

        //Use previous motor distance in calculation
        int distanceModified=distance+motorPosition;

        //Run while motor distance is less than target
        while (distanceModified>motorPosition && !isStopRequested()) {
            //set power to drive motors
            motorLeft.setPower(-Power);
            motorRight.setPower(Power);
            //update the average position of the motors
            motorPosition = (-motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
            //Telemetry for debugging
            telemetry.addData("motorPosition",motorPosition);
            telemetry.addData("Distance",distance);
            telemetry.update();
        }
        //Stops motors at power 0 if requested using the stop boolean variable
        if(stop) {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }
    public void DriveFwdAccDcc(double powerInitial, double powerFinal,int distance) throws InterruptedException {
        // Get current average motor position
        // Core hex motor with 90mm wheel is 0.03865148441 inches per count
        int motorPosition = (motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
        //Create a variable called powerCurrent for the current power
        double powerCurrent;
        //Use previous motor distance in calculation
        int distanceModified=distance+motorPosition;

        //Run while motor distance is less than target
        while (distanceModified>motorPosition) {
            //Set powerCurrent to the (changeInPower/distance)x(motorDistanceTraveled)
            //to make a linear increase in power.
            powerCurrent = ((powerFinal-powerInitial)/distance)*(distanceModified-motorPosition);
            //Set power to the motors
            motorLeft.setPower(-powerCurrent);
            motorRight.setPower(powerCurrent);
            //update the average position of the motors
            motorPosition = (motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
        }
    }

    public void DriveFwdObstacle(double Power,double thresholdPower){
        //Get average current motor speed and set it to motorEncoderSpeed Variable
        double motorEncoderSpeed= (motorRight.getPower()+motorLeft.getPower())/2;
        //While motor hasn't sensed resistance
        while (motorEncoderSpeed>thresholdPower && !isStopRequested()) {
            //Set power to the motors
            motorLeft.setPower(-Power);
            motorRight.setPower(Power);
            //update the average speed of the motors
            motorEncoderSpeed= (motorRight.getPower()+motorLeft.getPower())/2;
            //Telemetry for Debugging
            telemetry.addData("motorPower",motorEncoderSpeed);
            telemetry.addData("motorPowerThreshold",thresholdPower);
            telemetry.update();
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void TurnGyro (float degrees, float precision, double regurlarPower){
        //Get the gyro reading from the IMU
        float gyroReading =imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        //Adding previous value of the gyro to the target to find end position
        float totalDegees = degrees + gyroReading;
        //Log information for Debugging
        Log.i("GyroDebug", "CurrentAngle: " + gyroReading + "   target: " + totalDegees);


        //Set motors in opposite directions according to (+ or -)degrees
        double left = 1;
        double right= 1;
        //If degrees is negative = clockwise
        if(degrees<0){
            left =  -1;
            right= -1;
            //If degrees is Positive = anti-clockwise
        }else if(degrees>0){
            left = 1;
            right= 1;
        }
        //While the gyro is not greater or less than the target ...
        while(gyroReading>totalDegees+precision || gyroReading<totalDegees-precision){
            //turn the motors
            motorLeft.setPower(left*regurlarPower);
            motorRight.setPower(right*regurlarPower);
            //Update Gyro position
            gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
    }


}