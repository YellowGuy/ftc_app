package org.firstinspires.ftc.teamcode;

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

import java.util.List;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  frontleftDrive      = null;
    public DcMotor  frontrightDrive     = null;
    public DcMotor  backrightDrive      = null;
    public DcMotor  backleftDrive       = null;
    public DcMotor  arm                 = null;
    public DcMotor  out                 = null;
    public DcMotor  lift                = null;
    public Servo    left                = null;
    public Servo    right               = null;

    //Mid Position for Servo
    public static final double MID_SERVO       =  0.5 ;

    //Vuforia stuff
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "AQIw0CL/////AAABmVad/6kqB0Z5raIXZ2A27Y6G7TPsicTGxUU+GVOmNtC/8RLmGuFQM7q7Jb3EnSu1ocZvyXuEVH4Sm2NB55t8B7Zbu+PdFq3bxVypwOBdP95vv1p6bc1DPIGNkHN43kpVygzqw5qOWZMOHrurUxfrzlPF4dHIf5AORgQ2fhuJwVv0zUW2Ea1SaLneUbmeukI0CptofJhZB1lyHOy52EekBCZxjUdEeUSWRrGHS8+r8HdnO7atPM8jNugrPQk2PFKy7jAbED8ClOrcW9dElYNKmNc3thHcL13hwVWtryv7FmIylJJJ2H6iWJRnYMgmg5wx99W2Vml7fXpaltyLOcKqa2lBVT1jj+kIRDuYSFDGAr6e";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontleftDrive        = hwMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive       = hwMap.get(DcMotor.class, "front_right_drive");
        backleftDrive         = hwMap.get(DcMotor.class, "back_left_drive");
        backrightDrive        = hwMap.get(DcMotor.class, "back_right_drive");
        arm                   = hwMap.get(DcMotor.class, "arm");
        out                   = hwMap.get(DcMotor.class, "out");
        lift                  = hwMap.get(DcMotor.class, "lift");
        left                  = hwMap.get(Servo.class, "left");
        right                 = hwMap.get(Servo.class, "right");

        //Direction
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        arm.setDirection(DcMotor.Direction.FORWARD);
        out.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        arm.setPower(0);
        out.setPower(0);
        lift.setPower(0);
        left.setPosition(MID_SERVO);
        right.setPosition(MID_SERVO);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.

    }
 }

