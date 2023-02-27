package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoControllerEx;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
Class for controlling robot's virtual 4-bar. Includes
control for nub and movement of 4-bar. By Jake, 1/27/20.
 */

public class Arm2 {

    public Servo arm;
    public ServoControllerEx servoController;

    public LinearOpMode l;
    public Telemetry realTelemetry;

    @Config
    public static class ArmConstants {

        public static double arm_pickup = 0;
        public static double arm_low_front = 0.5; //0.5 0.37
        public static double arm_mid_front = 0.55;
        public static double arm_high_front = 0.55;
        public static double arm_high_back = 0.9;
        public static double arm_mid_back = 1;
        public static double arm_low_back = 0.9;
        public static  double arm_auto_dropoff = 0.5;
        public static double stack_top = 0.23;
        public static double stack_difference = 0.02;
        public static double auto_speed_slow = 0.0005;
        public static double auto_speed_fast = 0.01;
        public static double tolerance = 0.01;
        public static double teleop_speed = 0.01;
        public static double manual_speed = 0.006;

    }

    public enum armStatuses {
        PICKUP (ArmConstants.arm_pickup, ArmConstants.arm_pickup),
        LOW (ArmConstants.arm_low_front, ArmConstants.arm_low_back),
        MID (ArmConstants.arm_mid_front, ArmConstants.arm_mid_back),
        HIGH (ArmConstants.arm_high_front, ArmConstants.arm_high_back),
        AUTO (ArmConstants.arm_auto_dropoff, ArmConstants.arm_auto_dropoff);


        public double frontPosition;
        public double backPosition;
        armStatuses(double frontPos, double backPos) {
            this.frontPosition = frontPos;
            this.backPosition = backPos;
        }

        public double getFrontPosition() {return frontPosition;}
        public double getBackPosition() {return backPosition;}
    }

    public armStatuses armStatus = armStatuses.PICKUP;
    public double armPosition = 0;



    public Arm2(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;

        arm = hardwareMap.servo.get("armservo"); // Really not important which is which
        servoController = (ServoControllerEx) arm.getController();
        servoController.setServoPwmRange(arm.getPortNumber(),new PwmControl.PwmRange(900, 2100));

        arm.setPosition(armStatus.frontPosition); //sets current pos to 0


    }

    // Use this as auto function
    public void setArmPosition(armStatuses status, boolean Flip) {
        if (Flip){
            armPosition = status.getBackPosition();
        } else {
            armPosition = status.getFrontPosition();
        }
     //   arm.setPosition(armPosition);
    }

    public void setArmPositionSync(armStatuses status, boolean Flip) {
        if (Flip){
            armPosition = status.getBackPosition();
        } else {
            armPosition = status.getFrontPosition();
        }
           arm.setPosition(armPosition);
    }


    public void ArmStackControl (int cycle) {
        armPosition = ArmConstants.stack_top - (cycle* ArmConstants.stack_difference);
      //  arm.setPosition(armPosition);
    }

    public void ArmStackControlSync (int cycle) {
        armPosition = ArmConstants.stack_top - (cycle* ArmConstants.stack_difference);
        arm.setPosition(armPosition);
    }

    public void moveArm(double pos){
        double currentPos = arm.getPosition();
        while (currentPos <= pos-ArmConstants.tolerance || currentPos >= pos+ArmConstants.tolerance) {
            if (currentPos < pos){
                arm.setPosition(currentPos + ArmConstants.auto_speed_slow);
            } else if (currentPos > pos){
                arm.setPosition(currentPos - ArmConstants.auto_speed_slow);
            } else {
                break;
            }

            currentPos = arm.getPosition();
            realTelemetry.addData("armPos", currentPos);
        }


    }




    // Control Functions

//     Takes in toggles from main teleop code - flip is a toggle, all others trigger on press
    public void TeleopControl(boolean down, boolean low, boolean mid, boolean high, boolean flipToggle) {
        if (down){
            armStatus = armStatuses.PICKUP;
            armPosition = flipToggle ? (armStatus.getFrontPosition()) : armStatus.getBackPosition();
        }
        if (low){
            armStatus = armStatuses.LOW;
            armPosition = flipToggle ? (armStatus.getFrontPosition()) : armStatus.getBackPosition();
        }
        if (mid){
            armStatus = armStatuses.MID;
            armPosition = flipToggle ? (armStatus.getFrontPosition()) : armStatus.getBackPosition();
        }
        if (high){
            armStatus = armStatuses.HIGH;
            armPosition = flipToggle ? (armStatus.getFrontPosition()) : armStatus.getBackPosition();
        }



//        update();


        //TODO: Add manual control




    }

    public void ManualControl(double manualInput){
     //   if (armPosition >= ArmConstants.manual_speed || manualInput >= 0.0) {
            armPosition -= ArmConstants.manual_speed * manualInput;
     //   }
    }

    public void update() {
        double currentPos = arm.getPosition();
        if (currentPos <= armPosition-ArmConstants.tolerance || currentPos >= armPosition+ArmConstants.tolerance) {
            if (currentPos < armPosition) {
                arm.setPosition(currentPos + ArmConstants.teleop_speed);
            } else if (currentPos > armPosition) {
                arm.setPosition(currentPos - ArmConstants.teleop_speed);
            }
        }
        realTelemetry.addData("armPos", currentPos);
        realTelemetry.update();
    }

    public void autoUpdate() {
        double currentPos = arm.getPosition();
        if (currentPos <= armPosition-ArmConstants.tolerance || currentPos >= armPosition+ArmConstants.tolerance) {
            if (currentPos < armPosition) {
                arm.setPosition(currentPos + ArmConstants.auto_speed_fast);
            } else if (currentPos > armPosition) {
                arm.setPosition(currentPos - ArmConstants.auto_speed_fast);
            }
        }
        realTelemetry.addData("armPos", currentPos);
        realTelemetry.update();
    }

    public boolean isBusy() {
        double currentPos = arm.getPosition();
        return currentPos <= armPosition-ArmConstants.tolerance || currentPos >= armPosition+ArmConstants.tolerance;
    }



}