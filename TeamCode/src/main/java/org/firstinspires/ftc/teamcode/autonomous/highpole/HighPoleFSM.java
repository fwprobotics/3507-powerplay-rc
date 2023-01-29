package org.firstinspires.ftc.teamcode.autonomous.highpole;

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
import org.firstinspires.ftc.teamcode.subsystems.Lift2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class HighPoleFSM extends LinearOpMode {
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

    private STATE state;
    private enum STATE {
        START,
        TOSTACK,
        CYCLE,
        PARK,
        IDLE
    }
    
    //public HighPoleFSM(Field.autoZones z) {
        //zone = z;
   // }
    public void runOpMode() {

        telemetry.log().add("Select Corner RightRed(Up), LeftRed(Down), RightBlue(Right), LeftBlue(Left)");
        telemetry.update();
        Field.autoZones zone = null;
        while (zone == null) {
            
            if (gamepad1.dpad_up) zone = Field.autoZones.REDRIGHT;
            else if (gamepad1.dpad_down) zone = Field.autoZones.REDLEFT;
            else if (gamepad1.dpad_right) zone = Field.autoZones.BLUERIGHT;
            else if (gamepad1.dpad_left) zone = Field.autoZones.BLUELEFT;
        }
        telemetry.log().add("Selected" + zone.toString());
        telemetry.update();

        int xMult = 1;
        int yMult = 1;
        int headingMult = 1;
        double side = 140;
        switch (zone) {
            case REDRIGHT:
                xMult = 1;
                yMult = -1;
                headingMult = 1;
                side = 140;
                break;
            case REDLEFT:
                xMult = -1;
                yMult = -1;
                headingMult = 1;
                side = 40;
                break;
            case BLUERIGHT:
                xMult = -1;
                yMult = 1;
                headingMult = -1;
                side = 40;
                telemetry.log().add("Modifiers added");
                break;
            case BLUELEFT:
                xMult = 1;
                yMult = 1;
                headingMult = -1;
                side = 140;
                break;
        }

        telemetry.log().add("auton started");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift2 lift = new Lift2(hardwareMap, telemetry);
        Arm2 arm = new Arm2(this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap, telemetry);
        telemetry.clearAll();
        telemetry.log().add("hardware map read");
        telemetry.update();
        Pose2d startPose = new Pose2d(28.5*xMult, 66*yMult, Math.toRadians(90*headingMult));
        telemetry.log().add(startPose.toString());
        Field field = new Field(drive, 27.44/2.54, 25/2.54, zone);
        drive.setPoseEstimate(startPose);
        ElapsedTime matchTimer = new ElapsedTime();
        initCV();
        telemetry.log().add("CV inited");
        telemetry.update();
        arm.arm.setPosition(0);
        telemetry.log().add("Ready to go");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        claw.AutoControl(Claw.clawStatuses.CLOSED);
        sleep(500);
        matchTimer.reset();
        readSignal();
        TrajectorySequence startSequence = field.createFieldTrajectory(startPose)
                .toPole(0*xMult, 1*yMult, FieldTrajectorySequence.sides.RIGHT, true, true)
                .addMarker(() -> {
                    arm.setArmPosition(Arm2.armStatuses.HIGH, true);
                }, 1.5)
                .addMarker(() -> {
                    lift.setAutoPosition(Lift2.liftLevels.HIGH);
                }, 3.5)
                .build();


        TrajectorySequence toStackStart = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                .toStack(false, 5)

                .build();



        TrajectorySequence toPole =
                field.createFieldTrajectory(toStackStart.end())
                        .toPole(1*xMult, 0*yMult, side, false, true)
                        .build();
        TrajectorySequence toStack = field.createFieldTrajectory(toPole.end())
                .toStack(false, 0)
                .build();
        TrajectorySequence toParking = field.createFieldTrajectory(startSequence.end())
                //.toLocation(new Pose2d(toCone.end().getX(), toCone.end().getY(), Math.toRadians(180)), false)
                .addMarker(() -> {
                    lift.setHeight(0);

                }, 1)
                .addMarker(() -> {
                    arm.setArmPosition(Arm2.armStatuses.PICKUP, false);
                }, 3    )
                .toSignalZone(signalZone.zoneNum)
                .build();
        state = STATE.START;
        int cycle = 0;
        drive.followTrajectorySequenceAsync(startSequence);

      //  arm.setArmPosition(Arm2.armStatuses.HIGH, true);
        while (opModeIsActive() && !isStopRequested() && state != STATE.IDLE) {
            switch (state) {
                case START:
                    telemetry.addData("state", "START");
                    if (!drive.isBusy() && !lift.isBusy()) {
                        state = STATE.PARK;
                        sleep(500);
                        claw.AutoControl(Claw.clawStatuses.DROP);
                        sleep(500);
                        int finalCycle = cycle;
//                        toStackStart = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
//                                .toStack(false, finalCycle)
//                                .addMarker(() -> {
//                                    lift.setAutoPosition(Lift2.liftLevels.FLOOR);
//                                    arm.ArmStackControl(finalCycle);
//                                }, 1)
//                                .build();
                   //     cycle++;
                        drive.followTrajectorySequenceAsync(toParking);
                    }
                    break;
                case TOSTACK:
                    telemetry.addData("state", "TOSTACK");
                    if (!drive.isBusy()&& !lift.isBusy()) {
                        state = STATE.CYCLE;
                        claw.AutoControl(Claw.clawStatuses.CLOSED);
                        sleep(500);
                        lift.setAutoPosition(Lift2.liftLevels.HIGH);
                        arm.setArmPosition(Arm2.armStatuses.HIGH, false);
                        drive.followTrajectorySequenceAsync(toPole);
                    }
                    break;
                case CYCLE:
                    telemetry.addData("state", "CYCLE");
                    if (!drive.isBusy()&& !lift.isBusy()) {
                        if (cycle <= 4 && matchTimer.seconds() < 20) {
                            state = STATE.TOSTACK;
                            claw.AutoControl(Claw.clawStatuses.OPEN);
                            cycle++;
                            int finalCycle = cycle;
                            toStack = field.createFieldTrajectory(toPole.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                                    .toStack(false, finalCycle)
                                    .addMarker(() -> {
                                        lift.setAutoPosition(Lift2.liftLevels.FLOOR);
                                        arm.ArmStackControl(finalCycle);
                                    }, 500)
                                    .build();
                            drive.followTrajectorySequenceAsync(toStack);
                            arm.ArmStackControl(cycle);
                        } else {
                            state = STATE.PARK;
                            drive.followTrajectorySequenceAsync(toParking);
                            lift.setAutoPosition(Lift2.liftLevels.FLOOR);
                        }
                    }
                    break;
                case PARK:
                    telemetry.addData("state", "PARK");
                    if (!drive.isBusy()) {
                        state = STATE.IDLE;
                    }
                    break;
                case IDLE:
                    break;



            }

            drive.update();
            lift.update();
           // arm.update();
            telemetry.update();

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
