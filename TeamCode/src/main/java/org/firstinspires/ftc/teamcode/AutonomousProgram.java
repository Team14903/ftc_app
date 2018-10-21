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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


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

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Servos
        armServo = hardwareMap.servo.get("armServo");
        armServo.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            if (int t, t=0, t++) {
                DriveFwdDistance(100, 250, 5);
                Thread.sleep(5000, 0);
                DriveFwdDistance(100, 250, 5);
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
            //if statement cause robot to go half speed when 90% close to target or if slowDown is false
            if (distance < (distanceModified-motorPosition) * 0.5) {
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

}