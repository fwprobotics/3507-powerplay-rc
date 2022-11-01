package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicTeleOp extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor rightFront;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightDrive");

//        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
  //      leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            leftFront.setPower(gamepad1.left_stick_y);
            leftRear.setPower(gamepad1.left_stick_y);
            rightFront.setPower(gamepad1.right_stick_y);
            rightRear.setPower(gamepad1.right_stick_y);
            telemetry.addData("Front left", leftFront.getCurrentPosition());
            telemetry.addData("Front right", rightFront.getCurrentPosition());
            telemetry.addData("Back left", leftRear.getCurrentPosition());
            telemetry.addData("Back right", rightRear.getCurrentPosition());
            telemetry.update();
        }
    }
}
