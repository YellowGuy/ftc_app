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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
import java.math.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

@TeleOp(name="Driver_Control", group="Iterative Opmode")
//@Disabled
public class DriverControl extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();


    //Creates new robot
    HardwarePushbot robot       = new HardwarePushbot();


    //Gyro stuff
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    //needs to be outside main loop to not be redefined every time I think
    boolean manuelAngle = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        //I have zero clue what this does
        boolean lastResetState = false;
        boolean curResetState  = false;


        //Gets gyro from hardware map. This should eventually be moved to the Hardware Pushbot
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;


        //Initalize hardware from Hardware Pushbot
        robot.init(hardwareMap);


        //Calibrates the gyro
        modernRoboticsI2cGyro.calibrate();


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

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



        //I have zero clue what this does
        boolean lastResetState = false;
        boolean curResetState  = false;


        //Raw values are the angular rate of change
        int rawX = modernRoboticsI2cGyro.rawX();
        int rawY = modernRoboticsI2cGyro.rawY();
        int rawZ = modernRoboticsI2cGyro.rawZ();
        int heading = modernRoboticsI2cGyro.getHeading();
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();


        //Takes data from gyro to get angular velocity
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        //Creates integers for Z axis of gyro
        int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
        int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();


        //Gets z Values
        double angle = zAngle;

        if(gamepad1.a){
            manuelAngle = true;
        }else if(gamepad1.x){
            manuelAngle = false;
        }

        //Double Variables for driver control sticks
        double x  =  gamepad1.left_stick_x;
        double y  =  gamepad1.left_stick_y;
        double z  =  gamepad1.right_stick_x;


        //Arm and Out power doubles
        double rotate = gamepad2.left_stick_y;
        double extend = -gamepad2.right_stick_y;


        //Math for robot orientated drive. The Z axis offset is converted to radians. Then by multiplying the y and x values by the
        //cos and sin of the gyro, we can "rotate" the gamepad left stick, so forward on the sick is always away
        double pi = 3.14159286;
        double rad = angle * (pi/180);
        double forward = 0;
        double side = 0;
        if(manuelAngle){
            telemetry.addData(">", "Gyro-Driving is Disabled");
            forward = y;
            side = x;
        }else {
            telemetry.addData(">", "Gyro-Driving is Enabled");
            forward = (y * Math.cos(rad)) + (x * Math.sin(rad));
            side = (-y * Math.sin(rad)) + (x * Math.cos(rad));
        }
        telemetry.addData(">", "Heading = %f", zAngle);
        //telemetry.addData(">", "Heading = %d", modernRoboticsI2cGyro.getHeading());

        double Forward = Math.pow(forward,3);
        double Side = Math.pow(side,3);
        double Z = Math.pow(z,3);

        //Assigning drive power to motors
        /*robot.frontleftDrive.setPower(forward-side-z);
        robot.frontrightDrive.setPower(-forward-side-z);
        robot.backleftDrive.setPower(forward+side-z);
        robot.backrightDrive.setPower(-forward+side-z);*/
        robot.frontleftDrive.setPower(Forward-Side-Z);
        robot.frontrightDrive.setPower(-Forward-Side-Z);
        robot.backleftDrive.setPower(Forward+Side-Z);
        robot.backrightDrive.setPower(-Forward+Side-Z);

        //Assigning arm and out power
        robot.arm.setPower(rotate);
        robot.out.setPower(extend);


        //Lift Power
        if(gamepad2.y)
            robot.lift.setPower(1);
        else if (gamepad2.a)
            robot.lift.setPower(-1);
        else
            robot.lift.setPower(0);


        //Servo Position
       if (gamepad2.right_bumper){
           robot.left.setPosition(1);
           robot.right.setPosition(0);}
       else if (gamepad2.left_bumper){
           robot.left.setPosition(0);
           robot.right.setPosition(1);}
       else{
           robot.left.setPosition(.5);
           robot.right.setPosition(.5);
       }


        // If Y is pressed, the z will reset
        if (gamepad1.y)
            modernRoboticsI2cGyro.resetZAxisIntegrator();

        if (gamepad2.x)
            robot.left.setPosition(1);
        if (gamepad2.b)
            robot.left.setPosition(0);
        telemetry.update();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}