package org.firstinspires.ftc.teamcode.autonomous.lowpole;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ApriltagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class LowPoleFSM2 extends LinearOpMode {
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
        while (zone == null && !gamepad1.x) {

            if (gamepad1.dpad_up) zone = Field.autoZones.REDRIGHT;
            else if (gamepad1.dpad_down) zone = Field.autoZones.REDLEFT;
            else if (gamepad1.dpad_right) zone = Field.autoZones.BLUERIGHT;
            else if (gamepad1.dpad_left) zone = Field.autoZones.BLUELEFT;
        }
        telemetry.log().add("Selected" + zone.toString());
        telemetry.update();

        int xMult = 1;
        int yMult = 1;
        double xPoleOffset = 0;
        double yPoleOffset = 0;

        int headingMult = 1;
        double side = -90;
        int turnHeadingMult = 1;
        switch (zone) {
            case REDRIGHT:
                xMult = 1;
                yMult = -1;
                headingMult = 1;
                side = -90;
                xPoleOffset = -5;
                yPoleOffset = 1.5;
                turnHeadingMult = 1;
                break;
            case REDLEFT:
                xMult = -1;
                yMult = -1;
                headingMult = 1;
                side = -90;
                xPoleOffset = 0;
                yPoleOffset = 0;
                turnHeadingMult = -1;
                break;
            case BLUERIGHT:
                xMult = -1;
                yMult = 1;
                headingMult = -1;
                side = -90;
                telemetry.log().add("Modifiers added");
                break;
            case BLUELEFT:
                xMult = 1;
                yMult = 1;
                headingMult = -1;
                side = -90;
                break;
        }

        telemetry.log().add("auton started");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm2 arm = new Arm2(this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap, telemetry);
        telemetry.clearAll();
        telemetry.log().add("hardware map read");
        telemetry.update();
        Pose2d startPose = new Pose2d(29.5*xMult, 66*yMult, Math.toRadians(90*headingMult));
        telemetry.log().add(startPose.toString());
        Field field = new Field(drive, 27.44/2.54, 25/2.54, zone, telemetry);
        drive.setPoseEstimate(startPose);
        ElapsedTime matchTimer = new ElapsedTime();
        telemetry.log().add("Initing CV");
        initCV();
        telemetry.log().add("CV inited");
        telemetry.update();
        arm.arm.setPosition(0);
        telemetry.log().add("Ready to go");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        telemetry.log().add("Started");
        claw.AutoControl(Claw.clawStatuses.CLOSED);
        sleep(500);
        matchTimer.reset();
        telemetry.log().add("Reading the signal");
        readSignal();
        telemetry.log().add("Building traj");
        TrajectorySequence startSequence = field.createFieldTrajectory(startPose)
              //  .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL+0.5, DriveConstants.TRACK_WIDTH))
                //.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(15))
              //  .toLocation(new Pose2d(36*xMult, 0*yMult), false)
                .toPole(2*xMult, 1*yMult, side, false, true, new Pose2d(xPoleOffset, yPoleOffset))
                .addMarker(() -> {
                    arm.setArmPosition(Arm2.armStatuses.LOW, false);
                }, 0.5)
                .build();


        TrajectorySequence toStackStart = field.createFieldTrajectory(startSequence.end())
                .turn(90*turnHeadingMult)//new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                .toStack(false, 0)
                .addMarker(() -> {
                    arm.ArmStackControl(0);
                }, 0.5)
                .build();



        TrajectorySequence toPole =
                field.createFieldTrajectory(toStackStart.end())
                        .toPole(2*xMult, 1*yMult, side, false, true,new Pose2d(xPoleOffset, yPoleOffset))
                        .build();
        TrajectorySequence toStack = field.createFieldTrajectory(toPole.end())
                .turn(90*turnHeadingMult)
                .toStack(false, 0)
                .build();
        TrajectorySequence toParking = field.createFieldTrajectory(toPole.end())
                //.toLocation(new Pose2d(toCone.end().getX(), toCone.end().getY(), Math.toRadians(180)), false)
                .addMarker(() -> {
                    // arm.setArmPosition(Arm2.armStatuses.PICKUP, false);
                }, 1)
                .addMarker(() -> {
                    arm.setArmPositionSync(Arm2.armStatuses.PICKUP, false);
                }, 2    )
                .toSignalZone(signalZone.zoneNum)
                .build();
        state = STATE.START;
        int cycle = 0;
        telemetry.log().add("starting to drive");
        drive.followTrajectorySequenceAsync(startSequence);

        //  arm.setArmPosition(Arm2.armStatuses.HIGH, true);

        while (opModeIsActive() && !isStopRequested() && state != STATE.IDLE) {
            switch (state) {
                case START:
                    telemetry.addData("state", "START");
                    if (!drive.isBusy() && !arm.isBusy()) {
                        state = STATE.TOSTACK;
                        drive.setAngle(Math.toRadians(90*headingMult));
                        sleep(500);
                        claw.AutoControl(Claw.clawStatuses.OPEN);
                        sleep(500);

                        drive.followTrajectorySequenceAsync(toStackStart);
                        cycle++;
                    }
                    break;
                case TOSTACK:
                    telemetry.addData("state", "TOSTACK");
                    if (!drive.isBusy() && !arm.isBusy()) {
                        state = STATE.CYCLE;
                        telemetry.log().add("pick up pos "+drive.getPoseEstimate().toString());
                        drive.setAngle(Math.toRadians(90*headingMult));
                        claw.AutoControl(Claw.clawStatuses.CLOSED);

                        sleep(500);
                        arm.setArmPosition(Arm2.armStatuses.LOW, false);
                        sleep(500);
                        int finalCycle = cycle;
                        toPole =
                                field.createFieldTrajectory(drive.getPoseEstimate())
                                        .toPole(2*xMult, 1*yMult, side, false, true, new Pose2d(xPoleOffset, yPoleOffset))

                                        .build();
                        telemetry.log().add("pole start" + toPole.start().toString())
;                        drive.followTrajectorySequenceAsync(toPole);
                    }
                    break;
                case CYCLE:
                    telemetry.addData("state", "CYCLE");
                    if (!drive.isBusy() && !arm.isBusy()) {
                        if (cycle <= 4 && matchTimer.seconds() < 22) {
                            state = STATE.TOSTACK;
                            drive.setAngle(Math.toRadians(90*headingMult));
                            claw.AutoControl(Claw.clawStatuses.DROP);
                            sleep(500);
                            int finalCycle1 = cycle;

                            toStack = field.createFieldTrajectory(drive.getPoseEstimate())
                                    .turn(90*turnHeadingMult)
                                    .toStack(false, 0)
                                    .addMarker(() -> {
                                        arm.ArmStackControl(finalCycle1);
                                    }, 0.5)
                                    .addMarker(() -> {
                                        claw.AutoControl(Claw.clawStatuses.OPEN);
                                    }, 1)
                                    .build();
                            telemetry.log().add("tostackstart "+toStack.start().toString());
                            telemetry.log().add("tostackend "+toStack.end().toString());
                            drive.followTrajectorySequenceAsync(toStack);
                            cycle++;


                        } else {
                            state = STATE.PARK;
                            claw.AutoControl(Claw.clawStatuses.OPEN);
                            sleep(500);
                            drive.followTrajectorySequenceAsync(toParking);
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
            arm.autoUpdate();
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
