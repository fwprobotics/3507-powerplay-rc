package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Class for controlling robot's virtual 4-bar. Includes
control for nub and movement of 4-bar. By Jake, 1/27/20.
 */

public class Arm2 {

    public Servo arm;

    public LinearOpMode l;
    public Telemetry realTelemetry;

    @Config
    public static class ArmConstants {

        public static double arm_pickup = 0;
        public static double arm_low_front = 0.05;
        public static double arm_mid_front = 0.23;
        public static double arm_high_front = 0.45;
        public static double arm_high_back = 0.65;
        public static double arm_mid_back = 0.75;
        public static double arm_low_back = 0.75;

    }

    public enum armStatuses {
        PICKUP (ArmConstants.arm_pickup, ArmConstants.arm_pickup),
        LOW (ArmConstants.arm_low_front, ArmConstants.arm_low_back),
        MID (ArmConstants.arm_mid_front, ArmConstants.arm_mid_back),
        HIGH (ArmConstants.arm_high_front, ArmConstants.arm_high_back);


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


    public Arm2(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;

        arm = hardwareMap.servo.get("armservo"); // Really not important which is which

        //   arm.setPosition(armStatus.position());


    }

    // Use this as auto function
    public void setArmPosition(armStatuses status, boolean Flip) {
        if (Flip){
            arm.setPosition(status.getBackPosition());
        } else {
            arm.setPosition(status.getFrontPosition());
        }
    }


    // Control Functions

//     Takes in toggles from main teleop code - flip is a toggle, all others trigger on press
    public void TeleopControl(boolean down, boolean low, boolean mid, boolean high, boolean flipToggle) {
        if (down){
            armStatus = armStatuses.PICKUP;
        }
        if (low){
            armStatus = armStatuses.LOW;
        }
        if (mid){
            armStatus = armStatuses.MID;
        }
        if (high){
            armStatus = armStatuses.HIGH;
        }
        setArmPosition(armStatus, flipToggle);
    }

}