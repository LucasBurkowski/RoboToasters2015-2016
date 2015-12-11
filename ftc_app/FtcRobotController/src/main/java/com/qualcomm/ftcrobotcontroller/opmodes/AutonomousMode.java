package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/* Created by team 8487 on 11/29/2015.
 */
public class AutonomousMode extends OpMode {
    double[] left = {-1945, -978, -1196, 964,-989, -1925, -1149, 1456, -516};
    double[] right = {1945, -978, 1196, 964, 989, -1925, 1149, 1456, 516};

    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2;

    enum Mode {ResetEncoders, StartEncoders, Next, Moving, End}
    Mode mode;
    int moveState = 0;
    int threshold = 20;
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
            case StartEncoders:if(Math.abs(leftMotor.getCurrentPosition()) < 30 && Math.abs(rightMotor2.getCurrentPosition()) < 30) {
                    setEncoderState(DcMotorController.RunMode.RUN_USING_ENCODERS);
                    mode = Mode.Next;
                }else{
                    mode = Mode.ResetEncoders;
                }
                break;
            case Next:
                if(moveState < left.length) {
                    lTarget = left[moveState];
                    rTarget = right[moveState];
                    if (moveState % 2 == 0) {
                        kP = 0.0008;
                    } else {
                        kP = 0.004;
                    }
                    moveState++;
                    mode = Mode.Moving;
                }else{
                    mode = Mode.End;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            case Moving:
                if(!(Math.abs(lTarget - leftMotor.getCurrentPosition()) < threshold && Math.abs(rTarget - rightMotor2.getCurrentPosition()) < threshold  && leftMotor.getPower() == 0 && rightMotor2.getPower() == 0)) {
                    getSpeeds();
                    telemetry.addData("oh noes-ness", Math.abs(lTarget - leftMotor.getCurrentPosition()) + Math.abs(rTarget - rightMotor.getCurrentPosition()));
                }else{
                    mode = Mode.ResetEncoders;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            default:
                telemetry.addData("status", "finished");
                break;
        }
        runMotors();
        telemetry.addData("lPow", rightSpeed);
        telemetry.addData("lPos", rightMotor2.getCurrentPosition());
        telemetry.addData("diff", rTarget - rightMotor2.getCurrentPosition());
        telemetry.addData("mode", mode + " " + moveState);
    }
    void runMotors(){

        leftMotor.setPower(leftSpeed);
        rightMotor2.setPower(rightSpeed);
        leftMotor2.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }
    void setEncoderState(DcMotorController.RunMode r){
        leftMotor.setChannelMode(r);
        rightMotor2.setChannelMode(r);
    }
    void getSpeeds(){//calculate the motor speeeds
        leftSpeed = (lTarget - leftMotor.getCurrentPosition()) * kP;//set the proportional drivers
        rightSpeed = (rTarget - rightMotor2.getCurrentPosition()) * kP;
        limitValues();//limit the values
    }
    void limitValues(){
        if(leftSpeed > 0.4){//limit the values to 1
            leftSpeed = 0.4;
        }else if(leftSpeed < -0.4){
            leftSpeed = -0.4;
        }
        if(rightSpeed > 0.4){
            rightSpeed= 0.4;
        }else if(rightSpeed < -0.4){
            rightSpeed = -0.4;
        }
    }
}