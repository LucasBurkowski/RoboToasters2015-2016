package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.exceptions.UnexpectedI2CDeviceException;
import org.swerverobotics.library.interfaces.IBNO055IMU;

/* Created by team 8487 on 11/29/2015.
 */
public class ClimberDump extends OpMode {
<<<<<<< HEAD
    double[] dists = {-7656};
    double[] turns = {Math.PI * 3/4};
=======
    double[] dists = {-9200};
    double[] turns = {Math.PI};
>>>>>>> parent of 172abd5... Some Auto Progress
    float startAngle;
    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double target = 0;
    double lastTarget = 0;
    double integral = 0.05;
    double integralValue = 0;
    double startBrightness;
    double brightnessThreshold = 0.2;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2, allclear, allclear2;
    private I2cDevice gyro;
    private IBNO055IMU imu;
    private Servo plow;
    private Servo plow2;
    private AnalogInput lightSensor;
    boolean gyroActive=true;
    float angle;
    enum Mode {ResetEncoders, StartEncoders, Next, Moving, Turning, FindLine, End}
    Mode mode;
    int moveState = 0;
    int threshold = 10;
    double kP;
    public ClimberDump(){}
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
<<<<<<< HEAD
        lightSensor = hardwareMap.analogInput.get("Light");
=======
>>>>>>> parent of 172abd5... Some Auto Progress
        servoCont = hardwareMap.servoController.get("SrvCnt");
        climberThing = hardwareMap.servo.get("Srv");
        climberThing2 = hardwareMap.servo.get("Srv2");
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        mode = Mode.ResetEncoders;
        plow = hardwareMap.servo.get("Srv3");
        plow2 = hardwareMap.servo.get("Srv4");
        allclear = hardwareMap.servo.get("Srv5");
        allclear2 = hardwareMap.servo.get("Srv6");
        climberThing.setPosition(0.8);
        climberThing2.setPosition(0);
        allclear.setPosition(1);
        allclear2.setPosition(1);
        try {
            gyro = hardwareMap.i2cDevice.get("Gyro");
            imu = ClassFactory.createAdaFruitBNO055IMU(ClimberDump.this, gyro);
        }catch (UnexpectedI2CDeviceException e){
            gyroActive=false;
        }
        if(gyroActive){
            startAngle = (float) imu.getAngularOrientation().heading;
        }else{
            startAngle = 0;
        }
        plow.setPosition(1);
        plow2.setPosition(0);
<<<<<<< HEAD
        allclear.setPosition(0);
        allclear2.setPosition(1);
        startBrightness = lightSensor.getValue();
    }
    public void loop(){
        telemetry.addData("Brightness",lightSensor.getValue());
        switch(moveState){
            case 1:
                plow.setPosition(0.22);
                plow2.setPosition(0.74);
                break;
            //case 2:
                //plow.setPosition(1);
                //plow2.setPosition(0);
                //break;
            case 3:
                allclear.setPosition(1);
                allclear2.setPosition(0);
                break;
            default:
                break;
        }
=======
        allclear.setPosition(1);
        allclear2.setPosition(0);
    }
    public void loop(){
>>>>>>> parent of 172abd5... Some Auto Progress
        if(gyroActive) {
            angle = (float) imu.getAngularOrientation().heading - startAngle;
        }else{
            angle = 0;
            telemetry.addData("","oH NoES IT fAiLED");
        }
        //float angle = (float) 3.14;
        switch(mode) {
            case ResetEncoders:
                setEncoderState(DcMotorController.RunMode.RESET_ENCODERS);
                mode = Mode.StartEncoders;
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
                        kP = 0.002;
                        lastTarget = target;
                        target = dists[moveState / 2];
                        mode = Mode.Moving;
                    } else {
<<<<<<< HEAD
                        if (gyroActive) {
                            kP = 7;
=======
                        if(gyroActive) {
                            kP = 8;
>>>>>>> parent of 172abd5... Some Auto Progress
                            mode = Mode.Turning;
                            integral = 0.05;
                            integralValue = 0;
                            target = (turns[(moveState - 1) / 2] + Math.PI * 2) % (Math.PI * 2);//set the target, use remainder calculation to make it positive.
                        } else {
                            mode = Mode.Next;
                        }
                    }
                    switch(moveState){
                        case 0:
                            plow.setPosition(0.22);
                            plow2.setPosition(0.74);
                            break;
                        case 1:
                            plow.setPosition(1);
                            plow2.setPosition(0);
                            break;
                        case 2:
                            setAllClear(0,1000);
                            break;
                        default:
                            break;
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
                    getSpeeds(angle);
                    telemetry.addData("oh noes-ness", Math.abs(target - leftMotor.getCurrentPosition()) + Math.abs(target - rightMotor.getCurrentPosition()));
                } else {
                    mode = Mode.ResetEncoders;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            case Turning:
                if(Math.abs(angle - target) < Math.abs(angle - (target - Math.PI * 2))){
                    leftSpeed  = kP * (angle - target);
                    rightSpeed = kP * (angle - target);
                    integralValue += integral * (angle - target);
                    leftSpeed += integral;
                    rightSpeed += integral;
                }else{
                    leftSpeed  = kP * (angle - (target - Math.PI * 2));
                    rightSpeed = kP * (angle - (target - Math.PI * 2));
                    integralValue += integral * (angle - (target - Math.PI * 2));
                    leftSpeed += integral;
                    rightSpeed += integral;
                }
                limitValues();
                if (Math.abs(target - angle) < 0.1 || Math.abs(angle - (target - Math.PI * 2)) < 0.1){
                    mode = Mode.ResetEncoders;
                }
                break;
            case FindLine:
                leftSpeed = 0.3;
                rightSpeed = -0.3;
                if(lightSensor.getValue() > startBrightness + brightnessThreshold){
                    mode = Mode.Next;
                    leftSpeed = 0;
                    rightSpeed = 0;
                }
                break;
            default:
                telemetry.addData("",
<<<<<<< HEAD
                                "             ,,,,,\n" +
                                "           _|||||_\n" +
                                "       {~*~*~*~}\n" +
=======
                                "       ,,,,,\n" +
                                "      _|||||_\n" +
                                "     {~*~*~*~}\n" +
>>>>>>> parent of 172abd5... Some Auto Progress
                                "   __{*~*~*~*}__\n" +
                                "    `-------------`");
                break;
        }
        runMotors();
        telemetry.addData("angle", angle);
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
    void getSpeeds(double angle){//calculate the motor speeeds
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
    void setAllClear(double position, int loops){
        double difference = (allclear.getPosition() - position) / loops;
        for(double i = allclear.getPosition(); Math.abs(i - position) < 1E-4; i += difference){
            allclear.setPosition(i);
            allclear2.setPosition(1-i);
        }
        allclear.setPosition(position);
        allclear2.setPosition(1 - position);
    }
}
//the cake is a lie
//the cake is a lie
//the cake is a lie
//the cake is a lie
//the cake is t rue
//the cake is a lie
//the cake is a lie
//the cake is a lie
//the cake is a lie





//         ,,,,,
//        _|||||_
//       {~*~*~*~}
//     __{*~*~*~*}__
