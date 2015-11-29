package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;
/**
 * Created by Team 8487 on 11/8/2015.
 */
//Don't forget to mention that Spencer helped as well
public class TankDrivePlayBack extends OpMode{

    int potVolt = 0;
    int maxSpool = 10000000;
    int minSpool = 0;
    double servoPos = 0;
    double servoMin = 0;
    double servoMax = 0.9;
    float leftSpeed = 0;
    float rightSpeed = 0;
    float armPower = 0;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotorController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing;
    public File LeftRecord = new File ("LeftSpeed.txt");
    public File RightRecord = new File ("RightSpeed.txt");
    public File ArmRecord = new File ("ArmPower.txt");


    public void init(){//initializtion method, runs once at the beginning
        DcDrive = hardwareMap.dcMotorController.get("drive_controller");//find the motor controller on the robot
        DcDrive2 = hardwareMap.dcMotorController.get("drive_controller2");//find the motor controller on the robot
        cdi = hardwareMap.deviceInterfaceModule.get("cdi");
        ArmDrive = hardwareMap.dcMotorController.get("arm controller");
        leftMotor = hardwareMap.dcMotor.get("drive_left1");//find the motors on th robot
        rightMotor = hardwareMap.dcMotor.get("drive_right1");
        leftMotor2 = hardwareMap.dcMotor.get("drive_left2");
        rightMotor2 = hardwareMap.dcMotor.get("drive_right2");
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        pot = hardwareMap.analogInput.get("pot");
        servoCont = hardwareMap.servoController.get("SrvCnt");
        climberThing = hardwareMap.servo.get("Srv");
    }

    public void loop() {
        /*this section is the program that will continuously run while the robot is driving.*/
        try {//use try{to make it not fail if it doesnt work
            PlayBack();//plays back the prerecorded files
        } catch (IOException e){
            e.printStackTrace();
        }
        setMotors();
    }
    public void stop(){}

    public void PlayBack() throws IOException{

        try {
            Scanner LeftFile = new Scanner(LeftRecord);//read  the file
            Scanner RightFile = new Scanner(RightRecord);
            Scanner ArmFile = new Scanner(ArmRecord);
            if (LeftFile.hasNextLine()){
                leftSpeed = Float.parseFloat(LeftFile.nextLine());//set the speeds to the values read from the file
            }
            if (RightFile.hasNextLine()){
                rightSpeed = Float.parseFloat(RightFile.nextLine());
            }
            if (ArmFile.hasNextLine()){
                armPower = Float.parseFloat(RightFile.nextLine());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    void setMotors(){
        leftMotor2.setPower(leftSpeed);
        rightMotor2.setPower(rightSpeed);
        leftMotor.setPower(leftSpeed);//set the motors to the powers
        rightMotor.setPower(rightSpeed);
        //climberThing.setPosition(servoPos);
        //if((potVolt >= maxSpool && armPower >= 0) || (potVolt <= minSpool && armPower <= 0)){//If the arm is being moved ot of it range, dont move it.
            //arm1.setPower(0);
            //arm2.setPower(0);
        //}else{//other wise do move it.
            arm1.setPower(armPower);
            arm2.setPower(armPower);
        //}
    }

    double expo(double in, double amount){
        if (in > 0){
            return(Math.pow(in, amount));
        }else{
            return(-Math.pow(-in, amount));
        }
    }
}

