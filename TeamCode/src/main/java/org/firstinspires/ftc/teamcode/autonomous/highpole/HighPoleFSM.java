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
import org.firstinspires.ftc.teamcode.subsystems.Lift3;
import org.firstinspires.ftc.teamcode.subsystems.Lift3;
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
        int headingMult = 1;
        double side = 140;
        double startSide = FieldTrajectorySequence.sides.RIGHT.heading;
        double startXOffset = 0;
        double startYOffset = -1;
        double cycleXOffset = 0;
        double cycleYOffset = 1;
        double stackXOffset = 0;
        double stackYOffset = 0;
        switch (zone) {
            case REDRIGHT:
                xMult = 1;
                yMult = -1;
                headingMult = 1;
                side = 125;
                startSide = FieldTrajectorySequence.sides.RIGHT.heading;
                startXOffset = 1;
                startYOffset = -1;
                cycleXOffset = 1;
                cycleYOffset = 0;
                stackXOffset = 0;
                stackYOffset = 2;
                break;
            case REDLEFT:
                xMult = -1;
                yMult = -1;
                headingMult = 1;
                side = 55;
                startSide = FieldTrajectorySequence.sides.LEFT.heading;
                startXOffset = -1;
                startYOffset = 0;
                cycleXOffset = -3;
                cycleYOffset = 3;
                stackXOffset = 2;
                stackYOffset = 1.5;
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
        Lift3 lift = new Lift3(hardwareMap, telemetry, gamepad2);
        Arm2 arm = new Arm2(this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap, telemetry);
        telemetry.clearAll();
        telemetry.log().add("hardware map read");
        telemetry.update();
        Pose2d startPose = new Pose2d(29.5*xMult, 67*yMult, Math.toRadians(90*headingMult));
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
                .toPole(0*xMult, 1*yMult, startSide, true, true, new Pose2d(startXOffset, startYOffset))
                .addMarker(() -> {
                    arm.setArmPosition(Arm2.armStatuses.HIGH, true);
                }, 1.5)
                .addMarker(() -> {
                    lift.setAutoPosition(Lift3.liftLevels.HIGH);
                }, 3.5)
                .build();


        TrajectorySequence toStackStart = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                .toStack(false, 5)

                .build();



        TrajectorySequence toPole =
                field.createFieldTrajectory(toStackStart.end())
                        .toPole(1*xMult, 0*yMult, side, true, true, new Pose2d(0, 0))
                        .addMarker(() -> {
                            lift.setAutoPosition(Lift3.liftLevels.HIGH);
                        }, 3)
                        .build();
        TrajectorySequence toStack = field.createFieldTrajectory(toPole.end())
                .toStack(false, 0)
                .build();
        TrajectorySequence toParking = field.createFieldTrajectory(toPole.end())
                //.toLocation(new Pose2d(toCone.end().getX(), toCone.end().getY(), Math.toRadians(180)), false)
                .addMarker(() -> {
                    lift.setHeight(0);
                   // arm.setArmPosition(Arm2.armStatuses.PICKUP, false);
                }, 2)
                .addMarker(() -> {
                    arm.setArmPositionSync(Arm2.armStatuses.PICKUP, false);
                }, 0   )
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
                    if (!drive.isBusy() && !lift.isBusy() && !arm.isBusy()) {
                        state = STATE.TOSTACK;
                        drive.setAngle(Math.toRadians(90*headingMult));
                        sleep(0);
                        claw.AutoControl(Claw.clawStatuses.DROP);
                        int finalCycle = cycle;
                        int finalHeadingMult = headingMult;
                        toStackStart = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                                .toStack(false, finalCycle, new Pose2d(stackXOffset, stackYOffset))
                                .addMarker(() -> {
                                    lift.setAutoPosition(Lift3.liftLevels.FLOOR);
                                    arm.ArmStackControl(finalCycle);

                                }, 1.5)
                                .addMarker(() -> {
                                    drive.setAngle(Math.toRadians(90* finalHeadingMult));


                                }, 3)
                                .addMarker(() -> {
                                    claw.AutoControl(Claw.clawStatuses.OPEN);
                                }, 3)
                                .build();
                        cycle++;
                        drive.followTrajectorySequenceAsync(toStackStart);
                    }
                    break;
                case TOSTACK:
                    telemetry.addData("state", "TOSTACK");
                    if (!drive.isBusy()&& !lift.isBusy() && !arm.isBusy()) {
                        state = STATE.CYCLE;
                        drive.setAngle(Math.toRadians(90*headingMult));
                        claw.AutoControl(Claw.clawStatuses.CLOSED);

                        sleep(0);
                        arm.setArmPosition(Arm2.armStatuses.HIGH, true );
                        toPole =
                                field.createFieldTrajectory(toStack.end())
                                        .toPole(1*xMult, 0*yMult, side, true, true, new Pose2d(cycleXOffset, cycleYOffset))
                                        .addMarker(() -> {
                                            lift.setAutoPosition(Lift3.liftLevels.HIGH);
                                        }, 1)
                                        .build();
                        drive.followTrajectorySequenceAsync(toPole);
                    }
                    break;
                case CYCLE:
                    telemetry.addData("state", "CYCLE");
                    if (!drive.isBusy()&& !lift.isBusy() && !arm.isBusy()) {
                        if (cycle <= 4 && matchTimer.seconds() < 22) {
                            state = STATE.TOSTACK;
                            drive.setAngle(Math.toRadians(90*headingMult));
                            claw.AutoControl(Claw.clawStatuses.OPEN);
                            sleep(100);
                            cycle++;
                            int finalCycle = cycle;
                            toStack = field.createFieldTrajectory(toPole.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                                    .toStack(false, finalCycle, new Pose2d(stackXOffset, stackYOffset))
                                    .addMarker(() -> {
                                        lift.setAutoPosition(Lift3.liftLevels.FLOOR);
                                        arm.ArmStackControl(finalCycle);
                                    }, 0.5)
                                    .addMarker(() -> {


                                    }, 2)
                                    .addMarker(() -> {
                                        claw.AutoControl(Claw.clawStatuses.OPEN);
                                    }, 2.5)
                                    .build();
                            drive.followTrajectorySequenceAsync(toStack);
                            arm.ArmStackControl(cycle);
                        } else {
                            state = STATE.PARK;
                            claw.AutoControl(Claw.clawStatuses.OPEN);
                            sleep(200);
                            drive.followTrajectorySequenceAsync(toParking);
                            lift.setAutoPosition(Lift3.liftLevels.FLOOR);
                        }
                    }
                    break;
                case PARK:
                    telemetry.addData("state", "PARK");
                    if (!drive.isBusy() && !lift.isBusy() && !arm.isBusy()) {
                        state = STATE.IDLE;
                    }
                    break;
                case IDLE:
                    break;



            }

            drive.update();
            lift.update();
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
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
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
