package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Lucas on 12/1/2015.
 */
public class WayPoint extends CoordinateReader{

    int[] X = new int[50];
    int[] Y = new int[50];
    int[] RelativeX = new int[X.length];
    int[] RelativeY = new int[Y.length];

    public void GetCoordinateArray(){
        getCoordinates();
        X = GetX();
        Y = GetY();
        RelativeX = GetX();
        RelativeY = GetY();
        RelativeX = RelativeWaypoints(RelativeX, X);
        RelativeY = RelativeWaypoints(RelativeY, Y);
        RelativeX = ConvertCoordinate(RelativeX);
        RelativeY = ConvertCoordinate(RelativeY);
    }

    public int[] ConvertCoordinate(int[] Axis) {
        int Conversion = 78125 / 480; //The full length of the field in pixels/feet
        int[] finalArray = new int[Axis.length];
        for (int i = 0; i < Axis.length; i++) {
            finalArray[i] = (Axis[i] * Conversion);
        }
        return finalArray;
    }

    public int[] RelativeWaypoints(int[] Axis, int[] UnChanged) {
        int i = 0;
        if (Axis[0] == 0){
            i = 1;
        }
        else if (Axis[0] > 0){
            for (int j = 0; j < Axis.length-1; j++){
                Axis[j+1] = UnChanged[j+1] - UnChanged[j];
                if (j == 0) {
                    Axis[j] = 0;
                }
            }
        }
        return Axis;
    }

}
