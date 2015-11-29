package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by LeFevre on 11/21/2015.
 */
public class AutonTest extends OpMode {
    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double armPower = 0;
    double servoPos = 0;
    double servoPos2 = 0;
    double servoMin = 0, servoMin2 = .45;
    double servoMax = .45, servoMax2 = 0;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2;

    PID leftPID;
    PID alignPID;
    PID rightPID;

    double leftTarget, rightTarget;

    int state = 0;
    boolean moving = false;

    public AutonTest(){}//constructor

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
        servoCont = hardwareMap.servoController.get("SrvCnt");
        climberThing = hardwareMap.servo.get("Srv");
        climberThing2 = hardwareMap.servo.get("Srv2");

        servoPos = servoMax;//aet the servos to the up position
        servoPos2 = servoMax2;
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);//reverse some wheels

        leftPID = new PID(-0.001,0,0);
        rightPID = new PID(-0.001,0,0);
        alignPID = new PID(0.5, 0.01, 0.5);
    }

    public void loop(){

        if(!moving){
            switch(state){
                case 0:
                    leftTarget = 0;
                    rightTarget = 0;
                    leftPID.setTgt(-10000);
                    rightPID.setTgt(-10000);
                    moving = true;
                default:
                    stop();
            }
        }else{
            move();
        }
        telemetry.addData("posleft", leftMotor.getCurrentPosition());
        telemetry.addData("posR", rightMotor2.getCurrentPosition());
        telemetry.addData("rightPower", rightSpeed);
    }
    public void stop(){}
    void move(){
        leftSpeed = leftPID.run(leftMotor.getCurrentPosition());
        rightSpeed = rightPID.run(rightMotor2.getCurrentPosition());
        //alignPID.run(leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition());
        //leftSpeed += alignPID.outputVal;
        //rightSpeed += - alignPID.outputVal;
        if(leftSpeed > 1){
            leftSpeed = 1;
        }else if(leftSpeed < -1){
            leftSpeed = -1;
        }
        if(rightSpeed > 1){
            rightSpeed = 1;
        }else if(rightSpeed < -1){
            rightSpeed = -1;
        }
        leftMotor.setPower(leftSpeed);
        leftMotor2.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
        rightMotor2.setPower(rightSpeed);
        /*if(leftPID.getDistance() < 0.1 && rightPID.getDistance() < 0.1 && alignPID.getDistance() < 0.1){
            moving = false;
            state++;
            leftPID.reset();
            rightPID.reset();
            alignPID.reset();
            leftMotor.setPower(0);
            leftMotor2.setPower(0);
            rightMotor.setPower(0);
            rightMotor2.setPower(0);
        }*/
    }
}
