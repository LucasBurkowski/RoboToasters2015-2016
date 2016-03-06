package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.IBNO055IMU;
/**
 * Created by team 8487 on 11/8/2015.
 * //
 *
 *
*/
public class TankDrive extends OpMode {

    //double targetMult  = 1;

    double plowPos = 0;
    double plowCurrent = 0;

    double leftSpeed = 0;//variables for motor speeds
    double rightSpeed = 0;
    double turnExpo = 1.5;
    double speedExpo = 1.4;
    double armPower = 0;
    int potVolt = 0;
    double servoPos = 0;
    double servoPos2 = 0;
    double servoPos3 = 1;
    double servoPos4 = 0;
    double servoPos5 = 0;
    double servoPos6 = 1;
    double servoMin = 1, servoMin2 = 1;
    double servoMax = 0, servoMax2 = 0;
    int servoDeb = 30;
    private DcMotorController DcDrive, DcDrive2, ArmDrive;//create a DcMotoController
    private DcMotor leftMotor, rightMotor, leftMotor2, rightMotor2, arm1, arm2;//objects for the left and right motors
    private AnalogInput pot;
    private DeviceInterfaceModule cdi;
    private ServoController servoCont;
    private Servo climberThing,climberThing2,plow,plow2,allclear,allclear2;
    private I2cDevice gyro;
    private IBNO055IMU imu;
    public TankDrive(){}//constructor

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
        climberThing2 = hardwareMap.servo.get("Srv2");
        plow = hardwareMap.servo.get("Srv3");
        plow2 = hardwareMap.servo.get("Srv4");
        allclear = hardwareMap.servo.get("Srv5");
        allclear2 = hardwareMap.servo.get("Srv6");
        servoPos = servoMax;
        servoPos2 = servoMax2;
        gyro = hardwareMap.i2cDevice.get("Gyro");
        servoPos5 = 0;
        servoPos6 = 1;
        //imu = ClassFactory.createAdaFruitBNO055IMU(TankDrive.this, gyro);
        setEncoderState(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    public void loop() {
        /*this section is the program that will continuously run while the robot is driving.*/
        getInputs();
        mix();
        setMotors();
        //float angle = (float) imu.getAngularOrientation().heading;
        //float angle = (float) 3.14;
        //telemetry.addData("Angle", angle);
    }
    public void stop(){}
        //armPower = gamepad2.left_stick_y;
        //servoPos = (gamepad2.left_trigger != 0) ? servoMin : servoMax;
        //servoPos2 = (gamepad2.right_trigger !=0) ? servoMin : servoMax;
    void getInputs() {
        potVolt = pot.getValue();
        leftSpeed = gamepad1.left_stick_y;//get the inputs from the joystick
        rightSpeed = gamepad1.right_stick_y;
        armPower = gamepad2.left_stick_y;
        if (gamepad2.right_bumper && servoPos == servoMax && servoDeb > 30) {//toggle the servo when LB is pressed
            servoPos = servoMin;
            servoDeb = 0;
        }else if(gamepad2.right_bumper && servoDeb > 30){
            servoPos = servoMax;
            servoDeb = 0;
        }
        if (gamepad2.left_bumper && servoPos2 == servoMax2 && servoDeb > 30) {//toggle servo when RB is pressed
            servoPos2 = servoMin2;
            servoDeb = 0;
        }else if(gamepad2.left_bumper && servoDeb > 30){
            servoDeb = 0;
            servoPos2 = servoMax2;
        }
        if (gamepad2.a){
            plowPos = 0.95;
        }
        if (gamepad2.b){
            plowPos = 0;
        }
        if (gamepad2.y && plowPos != 0){
            servoPos5 = 0;
            servoPos6 = 1;
        }
        if (gamepad2.x && plowPos != 0){
            servoPos5 = 1;
            servoPos6 = 0;
        }
        if(Math.abs(plowCurrent - plowPos) > 1E-6){//since the math isn't exact sometimes, just get close
            if(plowCurrent > plowPos){
                plowCurrent -= 0.05;
            }else{
                plowCurrent += 0.05;
            }
        }else{//make it exact
            plowCurrent = plowPos;
        }
        telemetry.addData("plowPos", plowCurrent);
        servoPos3 = Range.clip(lerp(1, 0.22, plowCurrent), 0, 1);
        servoPos4 = Range.clip(lerp(0, 0.74, plowCurrent), 0, 1);
        if(gamepad1.left_stick_button && gamepad1.right_stick_button){//enable/disable encoders for teleop running
           setEncoderState(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            setEncoderState(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        servoDeb++;//servo debounce-- stops the servo variable from rapidly changing back and forth between up and down.
        telemetry.addData("encoders", leftMotor.getChannelMode());
    }
    void setMotors(){
        leftMotor2.setPower(leftSpeed);// set the motors to the powers
        leftMotor.setPower(leftSpeed);
        rightMotor2.setPower(rightSpeed);
        rightMotor.setPower(rightSpeed);
        climberThing.setPosition(servoPos);
        climberThing2.setPosition(servoPos2);
        plow.setPosition(servoPos3);
        plow2.setPosition(servoPos4);
        allclear.setPosition(servoPos5);
        allclear2.setPosition(servoPos6);

        //if((potVolt >= maxSpool && armPower >= 0) || (potVolt <= minSpool && armPower <= 0)){//If the arm is being moved ot of it range, dont move it.
            //arm1.setPower(0);
            //arm2.setPower(0);
        //}else{//other wise do move it.
            arm1.setPower(armPower);
            arm2.setPower(armPower);
       // }
        //this comment is useless :)
    }
    void setEncoderState(DcMotorController.RunMode r){
        leftMotor.setMode(r);
        rightMotor2.setMode(r);
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
    double lerp(double pos1, double pos2, double amount) {
        double difference = pos2 - pos1;
        difference *= amount;
        return(pos1 + difference);
    }
    double expo(double in, double amount){
        if (in > 0){
            return(Math.pow(in, amount));
        }else{
            return(-Math.pow(-in, amount));
        }
    }

}
