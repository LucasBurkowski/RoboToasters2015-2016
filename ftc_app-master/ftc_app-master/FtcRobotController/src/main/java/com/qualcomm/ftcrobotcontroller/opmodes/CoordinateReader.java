package com.qualcomm.ftcrobotcontroller.opmodes;

import android.os.Environment;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;

/**
 * Created by Lucas on 12/1/2015.
 */
public class CoordinateReader {
    int[] CoordinateX = new int[50];
    int[] CoordinateY = new int[50];

    public void getCoordinates(){
        int iterator = 0;
        StringTokenizer IntMaker;
        try{
            File CoordinateFile = new File(Environment.DIRECTORY_DOCUMENTS);
            File Coordinates = new File(CoordinateFile, "Coordinates.txt");

            BufferedReader Reader = new BufferedReader(new FileReader(Coordinates));
            String ReadLine;
            String X;
            String Y;
            while((ReadLine = Reader.readLine()) != null){
                ReadLine = Reader.readLine();
                IntMaker = new StringTokenizer(ReadLine);
                X = IntMaker.nextToken();
                Y = IntMaker.nextToken();
                CoordinateX[iterator] = Integer.parseInt(X);
                CoordinateY[iterator] = Integer.parseInt(Y);
            }
        }
        catch (IOException e){
            e.printStackTrace();
        }
    }

    public int[] GetX(){
        return CoordinateX;
    }

    public int[] GetY() {
        return CoordinateY;
    }
}
