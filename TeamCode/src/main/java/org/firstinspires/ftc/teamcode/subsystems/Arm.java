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

public class Arm {

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
        PICKUP (ArmConstants.arm_pickup),
        LOW_FRONT (ArmConstants.arm_low_front),
        MID_FRONT (ArmConstants.arm_mid_front),
        HIGH_FRONT (ArmConstants.arm_high_front),
        LOW_BACK (ArmConstants.arm_low_back),
        MID_BACK (ArmConstants.arm_mid_back),
        HIGH_BACK (ArmConstants.arm_high_back);


        public double position;
        armStatuses(double position) {this.position = position;}

        public double position() {return position;}
    }

    public armStatuses armStatus = armStatuses.PICKUP;


    public Arm(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;

        arm = hardwareMap.servo.get("armservo"); // Really not important which is which

     //   arm.setPosition(armStatus.position());


    }


    // AUTONOMOUS FUNCTIONS

    public void ArmAutoControl (armStatuses status){
        arm.setPosition(status.position());
    }
    // Control Functions

    // Takes in toggles from main teleop code - flip is a toggle, all others trigger on press
    public  void TeleopControl(boolean down, boolean low, boolean mid, boolean high, boolean flipToggle, boolean flipPress) {
        if (down) {
            armStatus = armStatuses.PICKUP;
        } else if (flipToggle) {
            if (low) {
                armStatus = armStatuses.LOW_BACK;
            } else if (mid) {
                armStatus = armStatuses.MID_BACK;
            } else if (high) {
                armStatus = armStatuses.HIGH_BACK;
            }
        } else if (low) {
            armStatus = armStatuses.LOW_FRONT;
        } else if (mid) {
            armStatus = armStatuses.MID_FRONT;
        } else if (high) {
            armStatus = armStatuses.HIGH_FRONT;
        } else if (flipPress) {
            switch (armStatus) {
                case LOW_BACK:
                    armStatus = armStatuses.LOW_FRONT;
                    break;
                case LOW_FRONT:
                    armStatus = armStatuses.LOW_BACK;
                    break;
                case MID_BACK:
                    armStatus = armStatuses.MID_FRONT;
                    break;
                case MID_FRONT:
                    armStatus = armStatuses.MID_BACK;
                    break;
                case HIGH_BACK:
                    armStatus = armStatuses.HIGH_FRONT;
                    break;
                case HIGH_FRONT:
                    armStatus = armStatuses.HIGH_BACK;
                    break;
                default:
                    break;
            }
        }
        arm.setPosition(armStatus.position());
    }

}