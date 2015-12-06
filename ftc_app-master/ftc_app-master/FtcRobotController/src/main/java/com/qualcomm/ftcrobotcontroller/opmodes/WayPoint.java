package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Lucas on 12/1/2015.
 */
public class WayPoint extends CoordinateReader{

    int[] X = new int[50];
    int[] Y = new int[50];
    CoordinateReader Reader = new CoordinateReader();
    int[] RelativeX = new int[X.length];
    int[] RelativeY = new int[Y.length];

    public void GetCoordinateArray(){
        Reader.getCoordinates();
        X = Reader.CoordinateX;
        Y = Reader.CoordinateY;
        RelativeWaypoints(RelativeX, Reader.CoordinateX);
        RelativeWaypoints(RelativeY, Reader.CoordinateY);
        ConvertCoordinate(RelativeX);
        ConvertCoordinate(RelativeY);
    }

    public int[] ConvertCoordinate(int[] Axis) {
        int Conversion = 1125 / 480; //The full length of the field in pixels/feet
        int[] finalArray = new int[Axis.length];
        for (int i = 0; i < Axis.length; i++) {
            Axis[i] = (Axis[i] * Conversion);
        }
        return finalArray;
    }

    public void RelativeWaypoints(int[] Axis, int[] UnChanged) {
        for (int i = 0; i < Axis.length-1; i++){
            if (i == 0){
                Axis[i] = 0;
                Axis[i+1] = UnChanged[i+1] - UnChanged[i];
            }
            if (i > 0) {
                Axis[i + 1] = UnChanged[i + 1] - UnChanged[i];
            }
        }
    }

}
