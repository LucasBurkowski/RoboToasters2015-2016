package com.qualcomm.ftcrobotcontroller.opmodes;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;


/**
 * Created by Lucas and team 8487
 */
public class CoordinateReader {
    StringBuilder text = new StringBuilder();
    int[] CoordinateX = new int[50];
    int[] CoordinateY = new int[50];
    String[] input = new String[2];
    String X;
    String Y;

    public void getCoordinates() {
        //int iterator = 0;
        CoordinateX[0] = 0;
        CoordinateY[0] = 0;
        CoordinateX[1] = 0;
        CoordinateY[1] = 480;
        //CoordinateX[2] = 200;
        //CoordinateY[2] = 200;
        /*
        try{
            File CoordinateFile = Environment.getExternalStorageDirectory();
            File Coordinates = new File(CoordinateFile, "Coordinates.txt");

            BufferedReader Reader = new BufferedReader(new FileReader(Coordinates));
            String ReadLine;
            while((ReadLine = Reader.readLine()) != null){
                text.append(ReadLine);
                text.append('\n');
                for (int i = 0; i < 7; i++){
                    if(ReadLine.charAt(i) == '\t'){
                        i++;
                    }
                    else if (i <= 3){
                        X = X + ReadLine.charAt(i);
                    }
                    else{
                        Y = Y + ReadLine.charAt(i);
                    }
                }
                CoordinateX[iterator] = Integer.parseInt(X);
                CoordinateY[iterator] = Integer.parseInt(Y);
                iterator++;
            }
            Reader.close();
        }
        catch (IOException e){
            e.printStackTrace();

        }
        */
    }

}
