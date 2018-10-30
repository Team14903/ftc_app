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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
public class AutonomousProgram extends LinearOpMode {

    //Declare Motor
    private DcMotor motorRight = null;
    private DcMotor motorLeft = null;
    //Declare Servo
    Servo armServo = null;
    //Define Gyro
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        int timesOpModeRun = 12;
        // Initialize motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Servos
        armServo = hardwareMap.servo.get("armServo");
        armServo.setPosition(0);

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
            if (timesOpModeRun>0) {
                DriveFwdAccDcc(0,100,250);
                DriveFwdDistance(100, 250, 0);
                DriveFwdAccDcc(100,0,250);
                Thread.sleep(5000, 0);
                TurnGyro(90,5,50,20);

                timesOpModeRun-=1;
            }
        }
    }
    // This is a method to make code easier to read, see above
    public void DriveFwdDistance(int Power,int distance,int slowDownPower) throws InterruptedException {
        // Get current motor position
        // (Core Hex Motor=1.25 Degrees/Count)
        // (HD Hex Motor=.32 Degrees/Count)
        int motorPosition = (motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;

        //Use previous motor distance in calculation
        int distanceModified=distance+motorPosition;

        //Run while motor distance is less than target
        while (distanceModified>motorPosition) {
            //if statement cause robot to go half speed when 90% close to target or if slowDown is 0
            if (distance < (distanceModified-motorPosition) * 0.5 || slowDownPower==0) {
                motorLeft.setPower(Power);
                motorRight.setPower(Power);
            } else{
                motorRight.setPower(Power / slowDownPower);
                motorLeft.setPower(Power / slowDownPower);
            }
            motorPosition = (motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
            telemetry.addData("motorPosition",motorPosition);
            telemetry.addData("Distance",distance);
            telemetry.update();
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
    public void DriveFwdAccDcc(int powerInitial, int powerFinal,int distance) throws InterruptedException {
        // Get current motor position
        // (Core Hex Motor=1.25 Degrees/Count)
        // (HD Hex Motor=.32 Degrees/Count)
        int motorPosition = (motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
        int powerCurrent;
        //Use previous motor distance in calculation
        int distanceModified=distance+motorPosition;

        //Run while motor distance is less than target
        while (distanceModified>motorPosition) {
            //Set powerCurrent to the (changeInPower/distance)(motorDistanceTraveled)
            powerCurrent = ((powerFinal-powerInitial)/distance)*(distanceModified-motorPosition);
            motorLeft.setPower(powerCurrent);
            motorRight.setPower(powerCurrent);
            motorPosition = (motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
    }
    public void TurnGyro (float degrees, float precision, int regurlarPower, int overshootPower){
        //Adding previous value of the gyro to the target to find end position
        float gyroReading =imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        float totalDegees = degrees + gyroReading;

        //Set motors in opposite directions according to (+ or -)degrees
        double left = 1;
        double right= 1;
        //If degrees is negative = clockwise
        if(degrees<0){
            left =  1;
            right= -1;
        }else if(degrees>0){
            left = -1;
            right=  1;
        }
        //While the gyro is not greater or less than the target ...
        while(!(gyroReading<totalDegees+precision)&&!(gyroReading>totalDegees-precision)){
            //If less than minimum,
            if (!(gyroReading>totalDegees-precision)){
                motorLeft.setPower(left*regurlarPower);
                motorRight.setPower(right*regurlarPower);
            } else if (!(gyroReading<totalDegees+precision)){
                motorLeft.setPower(left*overshootPower);
                motorRight.setPower(right*overshootPower);
            }
            gyroReading =imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
    }


}