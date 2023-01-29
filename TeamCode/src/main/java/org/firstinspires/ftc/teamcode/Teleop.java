package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Lift2;
import org.firstinspires.ftc.teamcode.subsystems.ToggleButton;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;


@TeleOp
public class Teleop extends LinearOpMode {

    SampleMecanumDrive drive;
    Drivetrain drivetrain;
    Lift2 lift;
    ToggleButton high;
    ToggleButton mid;
    ToggleButton low;
    ToggleButton pickup;
    ToggleButton flip;
    ToggleButton clawToggle;
    Claw claw;
    Arm2 arm;




    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this, hardwareMap, telemetry, drive);
        lift = new Lift2(hardwareMap, telemetry);
        high = new ToggleButton(false);
        mid = new ToggleButton(false);
        low = new ToggleButton(false);
        pickup = new ToggleButton(false);
        flip = new ToggleButton(false);
        clawToggle = new ToggleButton(false);
        claw = new Claw(hardwareMap, telemetry);
        arm = new Arm2(this, hardwareMap, telemetry);
        arm.arm.setPosition(0);
        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();

            while (opModeIsActive()) {

                drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_bumper, gamepad1.x);
                high.toggle(gamepad2.dpad_right);
                mid.toggle(gamepad2.dpad_up);
                low.toggle(gamepad2.dpad_left);
                pickup.toggle(gamepad2.dpad_down);
                flip.toggle(gamepad2.x);
                clawToggle.toggle(gamepad2.right_bumper);
                claw.TeleopControl(gamepad2.a, gamepad2.y, gamepad2.b);
                lift.teleOpControl(gamepad2);
//                arm.ManualControl(gamepad2.left_stick_y);
              //  if (high.newPress() || mid.newPress() || low.newPress() || pickup.newPress() || flip.newPress()) {
                    arm.TeleopControl(pickup.newPress(), low.newPress(), mid.newPress(), high.newPress(), true);
             //   }
                arm.ManualControl(gamepad2.left_stick_y);

                arm.update();
                telemetry.update();

            }
        }
    }
}