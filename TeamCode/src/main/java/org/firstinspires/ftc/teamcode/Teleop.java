package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@TeleOp
public class Teleop extends LinearOpMode {


    Drivetrain drivetrain;
    Lift lift;


    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        lift = new Lift(Lift.liftRunMode.AUTONOMOUS, this, hardwareMap, telemetry);
        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();

            while (opModeIsActive()) {

                drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_bumper, gamepad1.x);

                lift.teleOpControl(gamepad2.right_stick_y, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.dpad_down, gamepad2.dpad_up);
                telemetry.update();

            }
        }
    }
}