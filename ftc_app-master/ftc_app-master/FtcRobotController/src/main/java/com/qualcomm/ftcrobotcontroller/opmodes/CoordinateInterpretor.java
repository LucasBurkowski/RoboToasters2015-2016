package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.ModernRoboticsOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.Map;

/* Created by Lucas on 11/29/2015.
 */
public class CoordinateInterpretor extends OpMode{
    double turnsPerInch = 1;
    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double armPower = 0;
    double servoPos = 0;
    double servoPos2 = 0;
    double servoMin = 0, servoMin2 = .45;
    double servoMax = .45, servoMax2 = 0;
    double lastTurn, turn, distance;
    int wayPointNumber = 0;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2;
    WayPoint WayPoints = new WayPoint();

    enum Mode {ResetEncoders, StartEncoders, Next, Turning, Moving}
    Mode mode;
    int threshold = 10;
    double turnKP = 0.01;
    double straigtKP = 0.005;
    double lTarget = 0; double rTarget = 0;
    public CoordinateInterpretor(){}
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
        WayPoints.getCoordinates();
        WayPoints.GetCoordinateArray();
    }
    public void loop(){
        getRot_Dist();
        switch(mode){
            case ResetEncoders:
                setEncoderState(DcMotorController.RunMode.RESET_ENCODERS);
                mode = Mode.StartEncoders;
                break;
            case StartEncoders:
                if(leftMotor.getCurrentPosition() == 0 && rightMotor2.getCurrentPosition() ==0){
                    setEncoderState(DcMotorController.RunMode.RUN_USING_ENCODERS);
                    mode = Mode.Turning;
                    getRot_Dist();
                    lTarget = -turn * turnsPerInch;
                    rTarget = turn * turnsPerInch;
                }else{
                    mode = Mode.ResetEncoders;
                }
                break;
            case Turning:
                leftSpeed = lTarget - leftMotor.getCurrentPosition() * turnKP;
                rightSpeed = rTarget - rightMotor2.getCurrentPosition() * turnKP;
                runMotors();
                if(Math.abs(lTarget - leftMotor.getCurrentPosition()) < threshold && Math.abs(rTarget - rightMotor2.getCurrentPosition()) < threshold){
                    while(leftMotor.getCurrentPosition() != 0 && rightMotor2.getCurrentPosition() != 0){
                        setEncoderState(DcMotorController.RunMode.RESET_ENCODERS);
                    }
                    setEncoderState(DcMotorController.RunMode.RUN_USING_ENCODERS);
                    mode = Mode.Moving;
                    lTarget = distance;
                    rTarget = distance;
                }
                break;
            case Moving:
                leftSpeed = lTarget - leftMotor.getCurrentPosition() * turnKP;
                rightSpeed = rTarget - rightMotor2.getCurrentPosition() * turnKP;
                runMotors();
                if(Math.abs(lTarget - leftMotor.getCurrentPosition()) < threshold && Math.abs(rTarget - rightMotor2.getCurrentPosition()) < threshold){
                    mode = Mode.ResetEncoders;
                    wayPointNumber++;
                    telemetry.addData("WUAYPOIENT REEEECHD!!", "Wooo0ooo0o0ooo");
                }
                break;
            default:
                telemetry.addData("OH POOPIES", "SOMETHING HAPPENED");
        }
        telemetry.addData("mode:", mode);
    }
    void getRot_Dist(){
        double dX = WayPoints.RelativeX[wayPointNumber];
        double dY = WayPoints.RelativeY[wayPointNumber];
        double absoluteRotation;
        if(dX != 0) {
            if (dY >= 0) {
                absoluteRotation = 0;
            } else {
                absoluteRotation = Math.PI;
            }
        }else{
            absoluteRotation = Math.atan2(dX, dY);
        }
        turn = absoluteRotation - lastTurn;
        if(turn >= Math.PI){
            turn = turn - 2 * Math.PI;
        }
        distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
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

}