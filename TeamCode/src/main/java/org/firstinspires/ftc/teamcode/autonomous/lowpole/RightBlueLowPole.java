package org.firstinspires.ftc.teamcode.autonomous.lowpole;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ApriltagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.subsystems.FieldTrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

;

@Autonomous
public class RightBlueLowPole extends LinearOpMode {

    OpenCvCamera webcam;
    ApriltagDetectionPipeline pipeline;


    SignalZone signalZone;

    public enum SignalZone {
        LEFT (1), // A
        MIDDLE (2), // B
        RIGHT (3); // C
        public int zoneNum;
        SignalZone(int zoneN) {
            this.zoneNum = zoneN;
        }
    }

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
     //   Lift lift = new Lift(Lift.liftRunMode.AUTONOMOUS, this, hardwareMap, telemetry);
        Arm2 arm = new Arm2(this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap);
        Pose2d startPose = new Pose2d(-32, 66, Math.toRadians(-90));
        Field field = new Field(drive, 15, 16.5, Field.autoZones.BLUERIGHT);
        drive.setPoseEstimate(startPose);
        ElapsedTime matchTimer = new ElapsedTime();
        initCV();
        claw.AutoControl(Claw.clawStatuses.CLOSED);
        waitForStart();
        matchTimer.reset();
        readSignal();

//        TrajectorySequence startSequence = drive.trajectorySequenceBuilder(startPose)
//                .splineToSplineHeading(new Pose2d(66, -18, Math.toRadians(180)), Math.toRadians(0))
//                .build();

//        TrajectorySequence startSequence = drive.trajectorySequenceBuilder(startPose)
//                //.splineToConstantHeading(new Vector2d(30, -60), Math.toRadians(0))
//                .addTemporalMarker(0.5, () -> {
//                    arm.ArmAutoControl(Arm.armStatuses.LOW_BACK);
//        })
//                .splineToConstantHeading(new Vector2d(54, -63), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(54, -36), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(54, -20), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(52, -18, Math.toRadians(35)))
//
//                //.strafeTo(new Vector2d(12, -55))
//                // .lineToSplineHeading(new Pose2d(12, -24, Math.toRadians(180)))
//
//                .build();
        TrajectorySequence startSequence = field.createFieldTrajectory(startPose)
                .toPole(-2, 1, FieldTrajectorySequence.sides.DOWN, false, true)
                .build();


        Pose2d clearPoseEnd = field.createFieldTrajectory(startPose).getTargetPole(2, -1, FieldTrajectorySequence.sides.UP, false);
        TrajectorySequence toStack = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                .toStack(false)

                .build();

        TrajectorySequence toCone =
                field.createFieldTrajectory(toStack.end())
                        .toPole(-2, 1, FieldTrajectorySequence.sides.DOWN, false, true)
                        .build();
        TrajectorySequence toParking = field.createFieldTrajectory(toCone.end())
                //.toLocation(new Pose2d(toCone.end().getX(), toCone.end().getY(), Math.toRadians(180)), false)
                .toSignalZone(signalZone.zoneNum)
                .build();
//        TrajectorySequence clearPole = drive.trajectorySequenceBuilder(toCone.end())
//                .turn(Math.toRadians(90))
//                .build();

                //drive.trajectorySequenceBuilder(startSequence.end())
//                .setReversed(true)
//                .lineToConstantHeading(new Vector2d(9, -12), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
//                .forward(45)
//                .lineToLinearHeading(new Pose2d(60, -18, Math.toRadians(30)))
//
//                //.lineToConstantHeading(new Vector2d(60, -12))
//
//
//                .build();

        //TODO:Move lift up and extends arm to align with pole




//        TrajectorySequence fieldTrajectorySequence = field.createFieldTrajectory(startPose)
//                .toPole(2, -1, FieldTrajectorySequence.sides.UP, false, true)
//                .toStack(false)
//                .toPole(2, -1, FieldTrajectorySequence.sides.UP, false, false)
//                .toStack(false)
//                .toPole(2, -1, FieldTrajectorySequence.sides.UP, false, false)
//                .toSignalZone(1).build ();
        //   .toStack(false)

        arm.setArmPosition(Arm2.armStatuses.LOW, false);
       // lift.setPosition(Lift.dropoffOptions.HIGH);
        drive.followTrajectorySequence(startSequence);
        claw.AutoControl(Claw.clawStatuses.OPEN);
//        claw.AutoControl(Claw.clawStatuses.OPEN);
//        drive.followTrajectorySequence(toParking);
       // arm.ArmAutoControl(Arm.armStatuses.PICKUP);
       // lift.setPosition(Lift.dropoffOptions.FLOOR);

       // sleep(1000); //simulated cone dropping offing
        //TODO: Open claw to drop cone
       // drive.followTrajectorySequence(toStack);

        //this repeats until there is not enough time left to complete next cycle
        int cycle = 0;
        while (matchTimer.seconds() < 20 && opModeIsActive() && cycle <= 4) {
            sleep(500);
            //TODO: Move arm from stacj to pole and back
           // drive.followTrajectorySequence(clearPole);

            drive.followTrajectorySequence(toStack);
            arm.ArmStackControl(cycle);
           // claw.AutoControl(Claw.clawStatuses.OPEN);
            sleep(1000);
            claw.AutoControl(Claw.clawStatuses.CLOSED);
            sleep(500);
            arm.setArmPosition(Arm2.armStatuses.AUTO, false);
            sleep(1000);
            drive.followTrajectorySequence(toCone);
            claw.AutoControl(Claw.clawStatuses.OPEN);
//            claw.AutoControl(Claw.clawStatuses.OPEN);
//            claw.AutoControl(Claw.clawStatuses.CLOSED);
//            arm.ArmAutoControl(Arm.armStatuses.LOW_BACK);
//            claw.AutoControl(Claw.clawStatuses.DROP);
//
//            cycle++;

        }

        drive.followTrajectorySequence(toParking);
        arm.setArmPosition(Arm2.armStatuses.PICKUP, false);
        //TODO: lower lift retract arm
//        switch (signalZone) {
//            case LEFT:
//                drive.followTrajectorySequence(toLeftZone);
//                break;
//            case MIDDLE:
//                drive.followTrajectorySequence(toMiddleZone);
//                break;
//
//
//        }







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