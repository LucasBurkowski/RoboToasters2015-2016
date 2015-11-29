package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import java.io.*;
/**
 * Created by Team 8487 on 11/8/2015.
 */
//Don't forget to mention that Spencer helped as well
public class TankDriveRecording extends OpMode{

    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double turnExpo = 1.5;
    double speedExpo = 1.4;
    double armPower = 0;
    int potVolt = 0;
    int maxSpool = 10000000;
    int minSpool = 0;
    double servoPos = 0;
    double servoMin = 0;
    double servoMax = 0.9;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotorController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing;

    private boolean stopRec = false;

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
        getInputs();
        mix();
        setMotors();
        try {//use try{to make it not fail if it doesnt workbb
            Record();//try to record the values in a file
        } catch (IOException e){
            e.printStackTrace();
        }
        //telemetry.addData("potentiometer voltage", "pot power: " + String.format("%f", potVolt));
        //useless button section

    }
    public void stop(){}

    public void Record() throws IOException{
        try {
            FileWriter LeftWrite = new FileWriter(LeftRecord, false);//write the values to a file
            FileWriter RightWrite = new FileWriter(RightRecord, false);
            FileWriter ArmWrite = new FileWriter(ArmRecord, false);
            PrintWriter LeftWriter = new PrintWriter(LeftWrite);
            PrintWriter RightWriter = new PrintWriter(RightWrite);
            PrintWriter ArmWriter = new PrintWriter(ArmWrite);
            if(!stopRec) {
                LeftWriter.println(leftSpeed);
                RightWriter.println(rightSpeed);
                ArmWriter.println(armPower);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    void getInputs(){
        potVolt = pot.getValue();
        leftSpeed = gamepad1.left_stick_y;//get the inputs from the joysticksm
        rightSpeed = gamepad1.right_stick_y;
        armPower = gamepad2.left_stick_y;
        //servoPos = (gamepad2.left_trigger != 0) ? servoMin : servoMax;
        //if(gamepad1.left_bumper && gamepad1.right_bumper){
        //    stop = true;
        //}
    }
    void setMotors(){
        leftMotor2.setPower(leftSpeed);
        rightMotor2.setPower(rightSpeed);
        leftMotor.setPower(leftSpeed);//set the motors to the powers
        rightMotor.setPower(rightSpeed);
        climberThing.setPosition(servoPos);
        if((potVolt >= maxSpool && armPower >= 0) || (potVolt <= minSpool && armPower <= 0)){//If the arm is being moved ot of it range, dont move it.
            arm1.setPower(0);
            arm2.setPower(0);
        }else{//other wise do move it.
            arm1.setPower(armPower);
            arm2.setPower(armPower);
        }
    }
    void mix(){
        /*this mixing algorithm works by doing the following:
        * -mix the inputs into turn/speed
        * -apply some exponential stuff to each
        * -unmix the inputs into left/right again*/
        double turn = rightSpeed - leftSpeed;//calculate the amount the robot is turning
        double speed = (rightSpeed + leftSpeed)/2;//calculate the speed of the robot
        turn = expo(turn, turnExpo);//apply some exponential-ness. This makes small inputs smaller
        speed = expo(speed, speedExpo);
        double right = (turn + speed) / 2;
        double left = (-turn + speed) / 2;//un-mix the inputs
        left = Range.clip(left, -1, 1);//clip the range of the inputs to be within -1 and 1
        right = Range.clip(right, -1, 1);
        leftSpeed = left; rightSpeed = right;//set the speeds
    }
    double expo(double in, double amount){
        if (in > 0){
            return(Math.pow(in, amount));
        }else{
            return(-Math.pow(-in, amount));
        }
    }
}

