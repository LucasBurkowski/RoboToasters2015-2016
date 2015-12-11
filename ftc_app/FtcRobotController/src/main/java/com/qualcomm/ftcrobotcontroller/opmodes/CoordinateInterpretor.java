package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/* Created by Lucas on 11/29/2015.
 */
public class CoordinateInterpretor extends OpMode {
    double turnsPerInch = 1000;
    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double armPower = 0;
    double servoPos = 0;
    double servoPos2 = 0;
    double servoMin = 0, servoMin2 = .45;
    double servoMax = .45, servoMax2 = 0;
    double lastTurn, turn, distance;
    int wayPointNumber = 1;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2;
    WayPoint WayPoints = new WayPoint();
    double PixelsPerInch = 23.4375;

    enum Mode {ResetEncoders, StartEncoders, Next, Moving, END}
    Mode mode;
    int threshold = 20;
    double kP = 0.005;
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
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        mode = Mode.ResetEncoders;
        WayPoints.getCoordinates();
        WayPoints.GetCoordinateArray();
    }
    public void loop() {
        switch (mode) {
            case ResetEncoders:
                setEncoderState(DcMotorController.RunMode.RESET_ENCODERS);
                mode = Mode.StartEncoders;
                break;
            case StartEncoders:
                if (Math.abs(leftMotor.getCurrentPosition()) < 30 && Math.abs(rightMotor2.getCurrentPosition()) < 30) {
                    setEncoderState(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    mode = Mode.Next;
                } else {
                    mode = Mode.ResetEncoders;
                }
                break;
            case Next:
                if(wayPointNumber % 2 ==0){
                    getDist(wayPointNumber);
                    lTarget = distance;
                    rTarget = distance;
                }else{
                    getRot(wayPointNumber);
                    lTarget = 0;
                    rTarget = 0;
                }
                lTarget = WayPoints.CoordinateX[wayPointNumber];
                rTarget = WayPoints.CoordinateY[wayPointNumber];
                leftSpeed = 0;
                rightSpeed = 0;
                if(wayPointNumber < 49) {
                    wayPointNumber++;
                    mode = Mode.Moving;
                }else{mode = Mode.END;}
                break;
            case Moving:
                if (!(Math.abs(lTarget - leftMotor.getCurrentPosition()) < threshold && Math.abs(rTarget - rightMotor2.getCurrentPosition()) < threshold)) {
                    getSpeeds();
                    telemetry.addData("oh noes-ness", Math.abs(lTarget - leftMotor.getCurrentPosition()) + Math.abs(rTarget - rightMotor.getCurrentPosition()));
                } else {
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
        telemetry.addData("mode", mode);
    }
    void getDist(int num){
        if (wayPointNumber < 49) {
            double X1 = WayPoints.CoordinateX[num];
            double Y1 = WayPoints.CoordinateY[num];
            double X2 = WayPoints.CoordinateX[num + 1];
            double Y2 = WayPoints.CoordinateY[num + 1];
            double dX = (X2 - X1) * PixelsPerInch;
            double dY = (Y2 - Y1) * PixelsPerInch;
            distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
            distance = distance * PixelsPerInch;
        }
    }
    void getRot(int num){
        if (wayPointNumber < 49) {
            double X1 = WayPoints.CoordinateX[num];
            double Y1 = WayPoints.CoordinateY[num];
            double X2 = WayPoints.CoordinateX[num + 1];
            double Y2 = WayPoints.CoordinateY[num + 1];
            double dX = (X2 - X1) * PixelsPerInch;
            double dY = (Y2 - Y1) * PixelsPerInch;
            telemetry.addData("dx", dX + " dy: " + dY);
            double absoluteRotation;
            if (wayPointNumber > 0) {
                if (dX != 0 && dY != 0) {
                    if (dY >= 0) {
                        absoluteRotation = 0;
                    } else {
                        absoluteRotation = Math.PI;
                    }
                } else {
                    absoluteRotation = Math.atan2(dX, dY);
                }
                turn = absoluteRotation - lastTurn;
                if (turn >= Math.PI) {
                    turn = turn - 2 * Math.PI;
                }
            }
        }
    }
    void runMotors(){
        limitValues();
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