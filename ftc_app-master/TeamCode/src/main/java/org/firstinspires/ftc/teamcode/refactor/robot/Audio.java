package org.firstinspires.ftc.teamcode.refactor.robot;

import com.qualcomm.ftccommon.*;
import com.qualcomm.robotcore.eventloop.opmode.*;

import java.util.*;
import java.io.*;

public class Audio {
    private LinearOpMode opMode;
    public int depressionID;
    public int fitnessGramID;
    public int jasonBournID;
    public int theyAskYouID;
    public int[] IDs;

    public Audio(LinearOpMode opMode) {
        this.opMode = opMode;
        depressionID = opMode.hardwareMap.appContext.getResources().getIdentifier("crippling_depression", "raw", opMode.hardwareMap.appContext.getPackageName());
        fitnessGramID = opMode.hardwareMap.appContext.getResources().getIdentifier("fitness_gram", "raw", opMode.hardwareMap.appContext.getPackageName());
        jasonBournID = opMode.hardwareMap.appContext.getResources().getIdentifier("jason_bourne", "raw", opMode.hardwareMap.appContext.getPackageName());
        theyAskYouID = opMode.hardwareMap.appContext.getResources().getIdentifier("they_ask_you", "raw", opMode.hardwareMap.appContext.getPackageName());
        IDs = new int[] {depressionID, fitnessGramID, jasonBournID, theyAskYouID};
    }

    public void play(int index) {
        SoundPlayer.getInstance().startPlaying(opMode.hardwareMap.appContext, IDs[index]);
    }

    public void playRandom() {
        SoundPlayer.getInstance().startPlaying(opMode.hardwareMap.appContext, IDs[(int) (Math.random() * IDs.length)]);
    }
}
