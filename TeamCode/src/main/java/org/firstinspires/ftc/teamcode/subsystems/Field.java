package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
    public SampleMecanumDrive drive;
    private Telemetry telemetry;

    public Field(SampleMecanumDrive d, double rW, double rL, autoZones aZ, Telemetry realTelemetry) {
        width = rW;
        length = rL;
        autoZone = aZ;
        drive = d;
        telemetry = realTelemetry;
    }
    public FieldTrajectorySequence createFieldTrajectory(Pose2d startPose) {
        return new FieldTrajectorySequence(drive.trajectorySequenceBuilder(startPose), startPose,width, length, autoZone, telemetry);
    }
}
