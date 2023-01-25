package org.firstinspires.ftc.teamcode.autonomous.lowpole;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class LowPoleFSM extends LinearOpMode {
    OpenCvCamera webcam;
    ApriltagDetectionPipeline pipeline;


    RightRedLowPole.SignalZone signalZone;

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
    Field.autoZones zone;
    public LowPoleFSM(Field.autoZones z) {
        zone = z;
    }
    public void runOpMode() {
        int xMult = 1;
        int yMult = 1;
        int headingMult = 1;
        FieldTrajectorySequence.sides side = FieldTrajectorySequence.sides.UP;
        switch (zone) {
            case REDRIGHT:
                xMult = 1;
                yMult = -1;
                headingMult = 1;
                side = FieldTrajectorySequence.sides.UP;
                break;
            case REDLEFT:
                xMult = -1;
                yMult = -1;
                headingMult = 1;
                side = FieldTrajectorySequence.sides.UP;
            case BLUERIGHT:
                xMult = -1;
                yMult = 1;
                headingMult = -1;
                side = FieldTrajectorySequence.sides.DOWN;
            case BLUELEFT:
                xMult = 1;
                yMult = 1;
                headingMult = -1;
                side = FieldTrajectorySequence.sides.DOWN;

        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //   Lift lift = new Lift(Lift.liftRunMode.AUTONOMOUS, this, hardwareMap, telemetry);
        Arm2 arm = new Arm2(this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(32*xMult, 66*yMult, Math.toRadians(90*headingMult));
        Field field = new Field(drive, 15, 16.5, zone);
        drive.setPoseEstimate(startPose);
        ElapsedTime matchTimer = new ElapsedTime();
        initCV();
        claw.AutoControl(Claw.clawStatuses.CLOSED);
        waitForStart();
        matchTimer.reset();
        readSignal();
        TrajectorySequence startSequence = field.createFieldTrajectory(startPose)
                .toLocation(new Pose2d(12*xMult, 60*yMult, Math.toRadians(90*headingMult)), true)
                .toPole(2*xMult, 1*yMult, side, false, false)
                .build();


        TrajectorySequence toStack = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                .toStack(false, 1)

                .build();

        TrajectorySequence toPole =
                field.createFieldTrajectory(toStack.end())
                        .toPole(2*xMult, 1*yMult, side, false, true)
                        .build();
        TrajectorySequence toParking = field.createFieldTrajectory(toPole.end())
                //.toLocation(new Pose2d(toCone.end().getX(), toCone.end().getY(), Math.toRadians(180)), false)
                .toSignalZone(signalZone.zoneNum)
                .build();
        state = STATE.START;
        int cycle = 0;
        drive.followTrajectorySequenceAsync(startSequence);
        arm.setArmPosition(Arm2.armStatuses.LOW, false);
        while (opModeIsActive() && !isStopRequested() && state != STATE.IDLE) {
        switch (state) {
            case START:
                if (!drive.isBusy()) {
                    state = STATE.CYCLE;
                }
                break;
            case TOSTACK:
                if (!drive.isBusy()) {
                    state = STATE.CYCLE;
                    arm.setArmPosition(Arm2.armStatuses.LOW, false);
                    drive.followTrajectorySequenceAsync(toPole);
                }
                break;
            case CYCLE:
                if (!drive.isBusy()) {
                    if (cycle <= 4 && matchTimer.seconds() < 20) {
                        state = STATE.TOSTACK;
                        cycle++;
                        int finalCycle = cycle;
                        toStack = field.createFieldTrajectory(startSequence.end()) //new Pose2d(clearPoseEnd.getX(), clearPoseEnd.getY(), Math.toRadians(0))
                                .toStack(false, 1)
                                .addMarker(() -> {
                                    arm.ArmStackControl(finalCycle);
                                }, 500)
                                .build();
                        drive.followTrajectorySequenceAsync(toStack);
                        arm.ArmStackControl(cycle);
                    } else {
                        state = STATE.PARK;
                        drive.followTrajectorySequenceAsync(toParking);
                    }
                }
                break;
            case PARK:
                if (!drive.isBusy()) {
                    state = STATE.IDLE;
                }
                break;
            case IDLE:
                break;



        }

        drive.update();
        arm.update();

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
        if (signalzone == 1) signalZone = RightRedLowPole.SignalZone.LEFT;
        else if (signalzone == 2) signalZone = RightRedLowPole.SignalZone.MIDDLE;
        else if (signalzone == 3) signalZone = RightRedLowPole.SignalZone.RIGHT;
        else signalZone = signalZone.MIDDLE;
        webcam.closeCameraDevice();

    }
}
