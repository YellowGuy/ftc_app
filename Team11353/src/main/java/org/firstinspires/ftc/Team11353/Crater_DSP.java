package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
import java.math.*;
import java.util.List;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name= "Crater_DSP", group= "Crater")
//@Disabled
public class Crater_DSP extends LinearOpMode {

    //Selection Strings
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Vuforia Key
    public static final String VUFORIA_KEY = "AQIw0CL/////AAABmVad/6kqB0Z5raIXZ2A27Y6G7TPsicTGxUU+GVOmNtC/8RLmGuFQM7q7Jb3EnSu1ocZvyXuEVH4Sm2NB55t8B7Zbu+PdFq3bxVypwOBdP95vv1p6bc1DPIGNkHN43kpVygzqw5qOWZMOHrurUxfrzlPF4dHIf5AORgQ2fhuJwVv0zUW2Ea1SaLneUbmeukI0CptofJhZB1lyHOy52EekBCZxjUdEeUSWRrGHS8+r8HdnO7atPM8jNugrPQk2PFKy7jAbED8ClOrcW9dElYNKmNc3thHcL13hwVWtryv7FmIylJJJ2H6iWJRnYMgmg5wx99W2Vml7fXpaltyLOcKqa2lBVT1jj+kIRDuYSFDGAr6e";

    //Vuforia setup
    public VuforiaLocalizer vuforia;

    //Tfod setup
    public TFObjectDetector tfod;

    //New Robot
    HardwarePushbot robot       = new HardwarePushbot();

    //New Runtime
    private ElapsedTime runtime = new ElapsedTime();

    //Gyro Setup
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    @Override
    public void runOpMode() throws InterruptedException{

        //Init Hardware
        robot.init(hardwareMap);

        //Gyro setup
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        //Calibration
        //need to test if taking out the wait breaks anything
        modernRoboticsI2cGyro.calibrate();
        while (modernRoboticsI2cGyro.isCalibrating()){
            sleep(50);
            idle();
        }

        //Init Vuforia
        initVuforia();

        //Check for compatability in the camera
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        //Tells us its done init
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        //Wait for start
        waitForStart();
        while (opModeIsActive()){

            // don't think I need this with waitforstart()
        /*
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", modernRoboticsI2cGyro.getIntegratedZValue());
            telemetry.update();
        }*/
            //takes ages to start?


            //Drop off of Hook
        /* Eventually I would like to change this to a function that drops for a certain ammount
        of time until either a distance sensor reaches a certain distance off of the ground or a
        touch sensor hits the ground before unhooking. This would be more accurate, but for now
        I'm focusing on getting the selection mission done.
         */
            Lift(1850);

            //Activate Tfod
            tfod.activate();

            //Reverse off the Hook
            TimeDrive(10, 500, .25);


            //Move away from Lander
            TimeDrive(90, 500, 1);


            //Move to the third mineral

            TimeDrive(0, 450, .25);
            //Realign Robot
            //Turn(.5 , 250);
            sleep(250);

            //Drive until gold is found
        /*
        while(!findGold()){
            Drive(180 , .12);
        }
        TimeDrive(0  , 0250 , .5 );
        TimeDrive(90 , 1000 , .75);
        TimeDrive(270, 1000 , .75);
        idle();
        */


            for (int i = 0; i < 3; i++) {
                //Turn(.5, 100);
                int before = modernRoboticsI2cGyro.getHeading();

                //GTurn(0);
                int after = modernRoboticsI2cGyro.getHeading();

                //sleep(3000);
                if (findGold()) {
                    TimeDrive(90, 500, .75);
                    TimeDrive(270, 500, .75);
                }
                //GTurn(180);
                TimeDrive(195, 675, .55);
                /*telemetry.addData(">", "Before = %d", before);
                telemetry.addData(">", "After = %d", after);
                telemetry.addData(">", "After Drive = %d", modernRoboticsI2cGyro.getHeading());
                sleep(3000);
                telemetry.update();*/
            }
            TimeDrive(90, 900, .6);//accidentally used Depot finishing drive, replaced with Crater_DSP_2 finisher
            //get over to other side
     /*   TimeDrive(180, 4000,.75);
        //scan next 3 Minerals
        for(int i = 0; i < 3; i++){
            GTurn(180);
            if(findGold()){
                TimeDrive (180 , 1000 , .75);
                TimeDrive (0 , 1000 , .75);
            }
            //GTurn(90);
            Drive(90 , .12);
            sleep(500);
        }
*/

            stop();
        }
    }

    public void Arm (long time) throws InterruptedException {

        robot.arm.setPower(-1);
        sleep(time);
        robot.arm.setPower(0);

    }

    public void Lift(long time )throws InterruptedException{
        robot.lift.setPower(1);
        sleep(time);
        robot.lift.setPower(0);
        sleep(250);
    }

    public void TimeDrive (double angle , long time , double power) throws InterruptedException{

        double pi = 3.14159286;
        double rad = angle * (pi/180);
        double y = (Math.cos(rad));
        double x = (-Math.sin(rad));

        robot.frontleftDrive.setPower(power*(y-x));
        robot.frontrightDrive.setPower(power*(-y-x));
        robot.backleftDrive.setPower(power*(y+x));
        robot.backrightDrive.setPower(power*(-y+x));

        sleep(time);

        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);

        sleep(250);
        }

    public void Drive (double angle , double power){

        double pi = 3.14159286;
        double rad = angle * (pi/180);
        double y = (Math.cos(rad));
        double x = (-Math.sin(rad));

        robot.frontleftDrive.setPower(power*(y-x));
        robot.frontrightDrive.setPower(power*(-y-x));
        robot.backleftDrive.setPower(power*(y+x));
        robot.backrightDrive.setPower(power*(-y+x));

    }

    public void Turn (double power , long time) throws InterruptedException{
        robot.frontleftDrive.setPower(power);
        robot.frontrightDrive.setPower(power);
        robot.backleftDrive.setPower(power);
        robot.backrightDrive.setPower(power);

        sleep(time);

        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
    }

    public void GTurn (Integer targetAngle) throws InterruptedException{



        //Account for error in angle? - might act weird otherwise - something like:
        //  ((targetAngle + error) >! heading && (targetAngle - error) <! heading)
        //increase power? .5 didn't really work, others might, or might need error range for that


        while(targetAngle != modernRoboticsI2cGyro.getHeading()) {
            //see below for stackOverflow suggested answer
            //int m = ((x % 360) + 360) % 360
            //so adjHeading = heading + targetDif
            int targetDif = 180 - targetAngle; //so any angle will be x + targetDif if the targetAngle was 180
            //code at very end helps if the Heading + Dif < 0
            //modernRoboticsI2cGyro.getHeading()
            int adjHeading = (((modernRoboticsI2cGyro.getHeading() + targetDif) % 360) + 360) % 360;
            //telemetry.addData(">", "Heading = %d", modernRoboticsI2cGyro.getHeading());
            //telemetry.addData(">", "targetAngle = %d", targetAngle);
            //telemetry.addData(">", "targetDif = %d", targetDif);
            //telemetry.addData(">", "AdjHeading = %d", adjHeading);
            //telemetry.update();

            if (adjHeading > 180) {
                robot.frontleftDrive.setPower(-.1);
                robot.frontrightDrive.setPower(-.1);
                robot.backleftDrive.setPower(-.1);
                robot.backrightDrive.setPower(-.1);
            } else if (adjHeading < 180) {
                robot.frontleftDrive.setPower(.1);
                robot.frontrightDrive.setPower(.1);
                robot.backleftDrive.setPower(.1);
                robot.backrightDrive.setPower(.1);
            }
        }

        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);

        //Prev. Code
        /*
        while (targetAngle != modernRoboticsI2cGyro.getHeading()) {

            telemetry.addData(">", "Integrated Z Value = %d", modernRoboticsI2cGyro.getIntegratedZValue());
            telemetry.addData(">", "Heading = %d", modernRoboticsI2cGyro.getHeading());
            telemetry.update();
            if(targetAngle > modernRoboticsI2cGyro.getHeading()){
                robot.frontleftDrive.setPower(.1);
                robot.frontrightDrive.setPower(.1);
                robot.backleftDrive.setPower(.1);
                robot.backrightDrive.setPower(.1);
            }
            else if(targetAngle < modernRoboticsI2cGyro.getHeading()){
                robot.frontleftDrive.setPower(-.1);
                robot.frontrightDrive.setPower(-.1);
                robot.backleftDrive.setPower(-.1);
                robot.backrightDrive.setPower(-.1);
            }
        }

        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        */

    }

    public void GDrive (double angle , long time , double power , double z){

        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angel = zAngle;

    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public boolean findGold() {
        //Note: original had a bunch of redundancy for if it was missing TFod, didn't include
        //will return true if the gold mineral is seen
        boolean ans = false;

        //tfod.activate();

        sleep(3000);
        //sleep(1000);

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                ans = recognition.getLabel().equals(LABEL_GOLD_MINERAL);
            }
        }

        //tfod.shutdown();
        return ans;
    }

    public boolean findMineral(){
        //This checks for any mineral, not just gold
        //Will return true if a mineral is seen

        boolean min = false;

        //tfod.activate();

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null){
            min = true;
        }

        return min;
    }
}
