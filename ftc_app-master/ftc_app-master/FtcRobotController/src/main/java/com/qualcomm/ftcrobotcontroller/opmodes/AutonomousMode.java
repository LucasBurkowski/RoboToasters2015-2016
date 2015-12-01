package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

/* Created by team 8487 on 11/29/2015.
 */
public class AutonomousMode extends OpMode{
    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double armPower = 0;
    double servoPos = 0;
    double servoPos2 = 0;
    double servoMin = 0, servoMin2 = .45;
    double servoMax = .45, servoMax2 = 0;
    double turnP = 0.0001, turnI = 0.00001, turnD = 0.0001;
    double lastDiff = 0; double accum = 0;
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
                if(Math.abs(leftMotor.getCurrentPosition()) < 30 && Math.abs(rightMotor2.getCurrentPosition()) < 30) {
                    setEncoderState(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    mode = Mode.Next;
                }else{
                    mode = Mode.ResetEncoders;
                }
                break;
            case Next:
                switch(moveState){
                    case 0:
                        lTarget= -10000;
                        rTarget = 10000;
                        kP = 0.001;
                        threshold = 100;
                        break;
                    case 1:
                        lTarget = 10000;
                        rTarget = 10000;
                        kP = 0.01;
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
                if(!(Math.abs(lTarget - leftMotor.getCurrentPosition()) < threshold && Math.abs(rTarget - rightMotor2.getCurrentPosition()) < threshold)) {
                    getSpeeds();
                    telemetry.addData("oh noes-ness", Math.abs(lTarget - leftMotor.getCurrentPosition()) + Math.abs(rTarget - rightMotor.getCurrentPosition()));
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
        /*double lRatio = lTarget / leftMotor.getCurrentPosition(); double rRatio = rTarget / rightMotor2.getCurrentPosition();//find the ratio of the target position and the current position
        double diff = lRatio - rRatio;//find the difference in teh ratios
        accum += diff;//add to the accumulator for the intagral value
        double turnPIDOut = (diff) * turnP + (lastDiff - diff) * turnD + accum * turnI;//find thhe final value
        rightSpeed += turnPIDOut;
        leftSpeed -= turnPIDOut;
        limitValues();//limit th values*/
    }
    void limitValues(){
        if(leftSpeed > 1){//limit the values to 1
            leftSpeed = 1;
        }else if(leftSpeed < -1){
            leftSpeed = -1;
        }
        if(rightSpeed > 1){
            rightSpeed= 1;
        }else if(rightSpeed < -1){
            rightSpeed = -1;
        }
    }
}