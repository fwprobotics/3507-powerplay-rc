package org.firstinspires.ftc.teamcode.autonomous.ground;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ApriltagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

;

@Autonomous
public class LeftBlueGround extends LinearOpMode {

    OpenCvCamera webcam;
    ApriltagDetectionPipeline pipeline;


    SignalZone signalZone;

    public enum SignalZone {
        LEFT, // A
        MIDDLE, // B
        RIGHT // C
    }

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
     //   Lift lift = new Lift(Lift.liftRunMode.AUTONOMOUS, this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap);
        Pose2d startPose = new Pose2d(32, 66, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        ElapsedTime matchTimer = new ElapsedTime();
        initCV();
        claw.AutoControl(Claw.clawStatuses.CLOSED);
        waitForStart();

        if(isStopRequested()) return;
        matchTimer.reset();
        readSignal();


        TrajectorySequence startSequence = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .splineToConstantHeading(new Vector2d(49, 58), Math.toRadians(0))
                .build();

        TrajectorySequence toParking = drive.trajectorySequenceBuilder(startSequence.end())
                .lineToConstantHeading(new Vector2d(54, 57))
                .lineToConstantHeading(new Vector2d(54, 63))
                .lineToConstantHeading(new Vector2d(12, 63))
                .lineToConstantHeading(new Vector2d(12, 14))
                .build();
        TrajectorySequence toLeftZone = drive.trajectorySequenceBuilder(toParking.end())
                //       .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(60, 14))
                        .build();
        TrajectorySequence toMiddleZone = drive.trajectorySequenceBuilder(toParking.end())
            //    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(36, 14))
                .build();
//        TrajectorySequence toRightZone = drive.trajectorySequenceBuilder(toParking.end())
//
//                                        .lineToConstantHeading(new Vector2d(60, 12))
//                                                .build();
        //TODO:Move lift up and extends arm to align with pole

        drive.followTrajectorySequence(startSequence);
        claw.AutoControl(Claw.clawStatuses.OPEN);
        drive.followTrajectorySequence(toParking);


        //TODO: lower lift retract arm
        switch (signalZone) {
            case LEFT:
                drive.followTrajectorySequence(toLeftZone);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(toMiddleZone);
                break;
            case RIGHT:
               // drive.followTrajectorySequence(toRightZone);
                break;


        }






    }

    private void initCV() {
        // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Combines the above to create a webcam that we will use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Sets our pipeline to view images through as the one we want
        //(Boundary between regions 1 and 2, Boundary between 2 and 3, Far left, Far top, Far right, Far bottom, opmode, the side we're on)
        pipeline = new ApriltagDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        // Turns on the webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }
            //320 240

            //This is needed so it knows what to do if something goes wrong
            public void onError(int thing) {
                telemetry.addData("error", thing);
            }

        });

    }

    private void readSignal() {
        int signalzone = pipeline.getLastDetection();
        if (signalzone == 1) signalZone = SignalZone.LEFT;
        else if (signalzone == 2) signalZone = SignalZone.MIDDLE;
        else if (signalzone == 3) signalZone = SignalZone.RIGHT;
        else signalZone = signalZone.MIDDLE;
        webcam.closeCameraDevice();

    }
}


