package com.qualcomm.ftcrobotcontroller.opmodes;
import java.lang.String;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Lucas on 11/1/2015.
 */
public class AutonRed {

    double lPow =0;
    double rPow = 0;


    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;

    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing;
    public AutonRed(){}//constructor


    public void init(){//initializtion method, runs once at the beginning
        DcDrive = hardwareMap.dcMotorController.get("drive_controller");//find the motor controller on the robot
        DcDrive2 = hardwareMap.dcMotorController.get("drive_controller2");//find the motor controller on the robot
        cdi = hardwareMap.deviceInterfaceModule.get("cdi");
        ArmDrive = hardwareMap.dcMotorController.get("arm controller");
        leftMotor = hardwareMap.dcMotor.get("drive_left1");//find the motors on the robot
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

    }
    void move(String where, double speed){
        switch(where){
            case "forward":
                lPow = 1;
                rPow = 1;
            case "backward":
                lPow = -1;
                rPow = -1;
            case "left":
                lPow = -1;
                rPow = 1;
            case "right":
                lPow = 1;
                rPow = -1;
            case "Fright":
                lPow = 1;
                rPow = 0;
            case "Fleft":
                lPow = 0;
                rPow = 1;
            case "Bright":
                lPow = 0;
                rPow = -1;
            case "Bleft":
                lPow = -1;
                rPow = 0;
            default:
                lPow = 0;
                rPow = 0;
        }
        lPow = lPow * speed;
        rPow = rPow * speed;

    }
    public void stop(){}
}
