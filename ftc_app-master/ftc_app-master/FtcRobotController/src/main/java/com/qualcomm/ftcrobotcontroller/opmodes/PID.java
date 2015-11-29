package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by LeFevre on 11/21/2015.
 */
public class PID {
    double kP = 0;
    double kI = 0;
    double iAcc = 0;
    double kD = 0;
    double dDiff = 0;
    double tgt = 0;
    double cur = 0;
    double oldPos;
    double outputVal;
    public PID(double p, double i, double d){
        kP = p; kI = i; kD = d;
    }
    public void setTgt(double target){
        tgt = target;
    }
    public void reset(){
        iAcc = 0;
        dDiff = 0;
        cur = 0;
        oldPos = 0;
    }
    public double getDistance(){
        return(cur - tgt);
    }
    public double run(double currentPos) {
        oldPos = cur;
        cur = currentPos;
        iAcc += tgt - cur;//add to the integral value the distance off the target
        dDiff = oldPos - cur;
        outputVal = iAcc * kI;
        outputVal += dDiff * kD;
        outputVal += (tgt - cur) * kP;
        return(outputVal);
    }
}
