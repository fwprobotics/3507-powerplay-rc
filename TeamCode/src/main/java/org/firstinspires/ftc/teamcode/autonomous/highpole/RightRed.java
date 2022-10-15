package org.firstinspires.ftc.teamcode.autonomous.highpole;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.ApriltagDetectionPipeline;

@Autonomous
public class RightRed extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(30, -66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        ElapsedTime matchTimer = new ElapsedTime();
        initCV();
        waitForStart();
        matchTimer.reset();
        readSignal();

        TrajectorySequence startSequence = drive.trajectorySequenceBuilder(startPose)
                //.splineToConstantHeading(new Vector2d(30, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, -55), Math.toRadians(0))
        //.strafeTo(new Vector2d(12, -55))
               // .lineToSplineHeading(new Pose2d(12, -24, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(180)))
                .build();
        TrajectorySequence toStack = drive.trajectorySequenceBuilder(startSequence.end())
                .setReversed(true)
                //.lineToConstantHeading(new Vector2d(24, -14), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .strafeRight(12)
                //.lineToConstantHeading(new Vector2d(60, -12))
                .back(43)

                        .build();
//        TrajectorySequence toStackPartTwo = drive.trajectorySequenceBuilder(toStack.end())
//                .lineToConstantHeading(new Vector2d(60, -12))
//                .build();
        TrajectorySequence toPole = drive.trajectorySequenceBuilder(toStack.end())
                .forward(43)
                .strafeLeft(12)
                //.lineToConstantHeading(new Vector2d(12, -24))
                        .build();
        TrajectorySequence toLeftZone = drive.trajectorySequenceBuilder(toPole.end())
                .splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(0))
                        .build();
        TrajectorySequence toMiddleZone = drive.trajectorySequenceBuilder(toPole.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(0))
                .build();
//        TrajectorySequence toRightZone = drive.trajectorySequenceBuilder(toPole.end())
//                .splineTo(new Vector2d(60, -12), Math.toRadians(0))
//                .build();

        //TODO:Move lift up and extends arm to align with pole

        drive.followTrajectorySequence(startSequence);
        sleep(1000); //simulated cone dropping offing
        //TODO: Open claw to drop cone
//drive.followTrajectorySequence(toStack);
//drive.followTrajectorySequence(toStackPartTwo);
        //this repeats until there is not enough time left to complete next cycle
        while (matchTimer.seconds() < 20 && opModeIsActive()) {
            //TODO: Move lift to stack pos
            drive.followTrajectorySequence(toStack);
            sleep(1000); //simulated cone picking uping
            //TODO: Close claw, begin moving arm to pole pos
            drive.followTrajectorySequence(toPole);
            //TODO: Open claw to drop cone
            sleep(1000);

        }
        //TODO: lower lift retract arm
        switch (signalZone) {
            case LEFT:
                drive.followTrajectorySequence(toLeftZone);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(toMiddleZone);
                break;
            case RIGHT:
                drive.followTrajectorySequence(toStack);
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

    }
}



//    private void findCup() {
//
////Takes some time so we can run everything through the psipeline
//        sleep(3000);
//
//        LoopyPipeline2.Position cupPosition = pipeline.position;
//
////Sets cupPos to a corresponding position basesd on pipeline analysis
//
//        switch (cupPosition) {
//            case LEFT:
//                cupPos = CupPosition.LEFT;
//                break;
//            case MIDDLE:
//                cupPos = CupPosition.MIDDLE;
//                break;
//            case RIGHT:
//                cupPos = CupPosition.RIGHT;
//                break;
//        }
//
//    }
//}