package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift {

    public DcMotor leftLiftMotor;
    public DcMotor rightLiftMotor;
    public LinearOpMode l;
    public Telemetry realTelemetry;



    public enum liftRunMode {
        AUTONOMOUS,
        TELEOP
    }

    public enum dropoffOptions {
        FLOOR (0),
        LOW (0),
        MEDIUM (-960),
        HIGH (-1920);

        public int position;
        dropoffOptions(int position) {this.position = position;}

        public int position() {return position;}
    }



    @Config
    public static class LiftConstants {
        public static double power_modifier = 0.3;
        public static double auto_power_modifier = 0.3;
    }

    public Lift(liftRunMode runmode, LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;

        leftLiftMotor = hardwareMap.dcMotor.get("leftLiftMotor");
        rightLiftMotor = hardwareMap.dcMotor.get("rightLiftMotor");

        // Different motor configurations depending on use case
        switch (runmode) {
            case AUTONOMOUS:
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setTargetPosition(0);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               // leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setTargetPosition(0);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case TELEOP:
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

        }
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse left side

    }



    public void setAutoPosition(dropoffOptions Pos){
        setPosition(Pos);
        while (leftLiftMotor.isBusy() || rightLiftMotor.isBusy()) {
            l.idle();
        }
    }


    public void setPosition(dropoffOptions Pos){
        leftLiftMotor.setTargetPosition(Pos.position());
        rightLiftMotor.setTargetPosition(Pos.position());
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPower(LiftConstants.auto_power_modifier);
        rightLiftMotor.setPower(LiftConstants.auto_power_modifier);
        if (Pos.position() == 0) {
            leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void teleOpControl(double input, boolean high, boolean low, boolean floor, boolean medium) { // Rename inputs based on real buttons we choose

        if (floor) {
            setPosition(dropoffOptions.FLOOR);
        }
        if (high) {
            setPosition(dropoffOptions.HIGH);
        }
        if (medium) {
            setPosition(dropoffOptions.MEDIUM);
        }
        if (low) {
            setPosition(dropoffOptions.LOW);
        }
        //if its not going to any position and the motors aren't busy or the run position is RUN_USING_ENCODER
        if (!floor & !high & !medium & !low & ((!leftLiftMotor.isBusy() & !rightLiftMotor.isBusy()) || leftLiftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)) {

            leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //keeps lift within range
            if ((input < 0 & leftLiftMotor.getCurrentPosition() >= dropoffOptions.HIGH.position()) | (input > 0 & leftLiftMotor.getCurrentPosition() <= 0) ) {
                leftLiftMotor.setPower(input * LiftConstants.power_modifier);
                rightLiftMotor.setPower(input * LiftConstants.power_modifier); //Probably should/can get toned down
            } else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);
            }
        }

        l.telemetry.addData("left encoder", leftLiftMotor.getCurrentPosition());
        l.telemetry.addData("right encoder", rightLiftMotor.getCurrentPosition());

    }
}