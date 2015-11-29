package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
/* Created by Lucas on 11/29/2015.
 */
public class AutonomousMode extends OpMode{
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

    enum Mode {ResetEncoders, StartEncoders, Next, Moving}
    Mode mode;
    int moveState = 0;
    int threshold = 10;
    double kP;
    double lTarget = 0; double rTarget = 0;
    public AutonomousMode(){}
    public void init(){
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
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        mode = Mode.ResetEncoders;
    }
    public void loop(){
        switch(mode){
            case ResetEncoders:
                setEncoderState(DcMotorController.RunMode.RESET_ENCODERS);
                mode = Mode.StartEncoders;
                break;
            case StartEncoders:
                setEncoderState(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                mode = Mode.Next;
                break;
            case Next:
                switch(moveState){
                    case 0:
                        lTarget= -10000;
                        rTarget = 10000;
                        kP = 0.0003;
                        threshold = 100;
                        break;
                    default:
                        lTarget = 0;
                        rTarget = 0;
                        kP = 0;
                        threshold = -1;
                }
                moveState++;
                mode = Mode.Moving;
                break;
            case Moving:
                if(!(Math.abs(lTarget - leftMotor.getCurrentPosition()) < threshold && Math.abs(rTarget - rightMotor.getCurrentPosition()) < threshold)) {
                    leftSpeed = (lTarget - leftMotor.getCurrentPosition()) * kP;
                    rightSpeed = (rTarget - rightMotor2.getCurrentPosition()) * kP;
                }else{
                    mode = Mode.ResetEncoders;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            default:
                telemetry.addData("OH NOES", "IT FAILD");
                break;
        }
        runMotors();
        telemetry.addData("lPow", rightSpeed);
        telemetry.addData("lPos", rightMotor2.getCurrentPosition());
        telemetry.addData("diff", rTarget - rightMotor2.getCurrentPosition());
    }
    void runMotors(){
        if(leftSpeed > 1){
            leftSpeed = 1;
        }else if(leftSpeed < -1){
            leftSpeed = -1;
        }
        if(rightSpeed > 1){
            rightSpeed= 1;
        }else if(rightSpeed < -1){
            rightSpeed = -1;
        }
        leftMotor.setPower(leftSpeed);
        rightMotor2.setPower(rightSpeed);
        leftMotor2.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }
    void setEncoderState(DcMotorController.RunMode r){
        leftMotor.setChannelMode(r);
        rightMotor2.setChannelMode(r);
    }
}