package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Field {
    public static int border = 6;


    public enum autoZones {
        REDRIGHT,
        REDLEFT,
        BLUERIGHT,
        BLUELEFT
    }
    public double width;
    public double length;
    public autoZones autoZone;
    public DriveShim drive;

    public Field(DriveShim d,  double rW, double rL, autoZones aZ) {
        width = rW;
        length = rL;
        autoZone = aZ;
        drive = d;
    }
    public FieldTrajectorySequence createFieldTrajectory(Pose2d startPose) {
        return new FieldTrajectorySequence(drive.trajectorySequenceBuilder(startPose), startPose,width, length, autoZone);
    }
}
