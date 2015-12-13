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
    double[] dists = {-6251, 3513, -3513, -1000};
    double[] turns = {3.14, 3.14, 3.14, 3.14};
    float startAngle;
    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double target;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2;
    //private I2cDevice gyro;
    //private IBNO055IMU imu;
    private Servo plow;
    private Servo plow2;
    enum Mode {ResetEncoders, StartEncoders, Next, Moving, Turning, End}
    Mode mode;
    int moveState = 0;
    int threshold = 20;
    double kP;
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
        plow = hardwareMap.servo.get("Srv3");
        plow2 = hardwareMap.servo.get("Srv4");
        //gyro = hardwareMap.i2cDevice.get("Gyro");
        //imu = ClassFactory.createAdaFruitBNO055IMU(AutonomousMode.this, gyro);
        //startAngle = (float) imu.getAngularOrientation().heading;
    }
    public void loop(){
        //float angle = (float) imu.getAngularOrientation().heading - startAngle;
        float angle = (float) 3.14;
        switch(mode) {
            case ResetEncoders:
                setEncoderState(DcMotorController.RunMode.RESET_ENCODERS);
                mode = Mode.StartEncoders;
                plow.setPosition(0.22);
                plow2.setPosition(0.74);
                break;
            case StartEncoders:
                if (Math.abs(leftMotor.getCurrentPosition()) < 30 && Math.abs(rightMotor2.getCurrentPosition()) < 30) {
                    setEncoderState(DcMotorController.RunMode.RUN_USING_ENCODERS);
                    mode = Mode.Next;
                } else {
                    mode = Mode.ResetEncoders;
                }
                break;
            case Next:
                if (moveState < dists.length * 2) {
                    if (moveState % 2 == 0) {
                        kP = 0.0008;
                        target = dists[moveState / 2];
                        mode = Mode.Moving;
                    } else {
                        kP = 1;
                        mode = Mode.Turning;
                        target = turns[(moveState - 1) / 2];
                    }
                    moveState++;
                } else {
                    mode = Mode.End;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            case Moving:
                if (!(Math.abs(-target - leftMotor.getCurrentPosition()) < threshold && Math.abs(target - rightMotor2.getCurrentPosition()) < threshold && leftMotor.getPower() == 0 && rightMotor2.getPower() == 0)) {
                    getSpeeds();
                    telemetry.addData("oh noes-ness", Math.abs(target - leftMotor.getCurrentPosition()) + Math.abs(target - rightMotor.getCurrentPosition()));
                } else {
                    mode = Mode.ResetEncoders;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            case Turning:
                if(angle - target < Math.PI){
                    leftSpeed = -kP * (angle - target);
                    rightSpeed = -kP * (angle - target);
                }else {
                    leftSpeed = kP * (angle - target);
                    rightSpeed = kP * (angle - target);
                }
                limitValues();
                if (Math.abs(target - angle) < 0.05){
                    mode = Mode.ResetEncoders;
                }
                break;
            default:
                telemetry.addData("status", "finished");
                break;
        }
        runMotors();
        telemetry.addData("lPow", rightSpeed);
        telemetry.addData("lPos", rightMotor2.getCurrentPosition());
        telemetry.addData("diff", target - rightMotor2.getCurrentPosition());
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
        leftSpeed = (-target - leftMotor.getCurrentPosition()) * kP;//set the proportional drivers
        rightSpeed = (target - rightMotor2.getCurrentPosition()) * kP;
        limitValues();//limit the values
    }
    void limitValues(){
        if(leftSpeed > 0.3){//limit the values to 1
            leftSpeed = 0.3;
        }else if(leftSpeed < -0.3){
            leftSpeed = -0.3;
        }
        if(rightSpeed > 0.3){
            rightSpeed= 0.3;
        }else if(rightSpeed < -0.3){
            rightSpeed = -0.3;
        }
    }

}