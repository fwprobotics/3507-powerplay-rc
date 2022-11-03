package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//This is quite self explanatory - just moves a servo between positions based on if we want the claw open or not
public class Claw {
    public static Servo ClawServo;

    @Config
    public static class ClawPositions {
        public static double open = 0;
        public static double closed = 1;
    }

    public enum clawStatuses {
        CLOSED (ClawPositions.closed),
        OPEN (ClawPositions.open);

        public double position;
        clawStatuses(double position) {this.position = position;}

        public double position() {return position;}
    }

    public Claw(HardwareMap hardwareMap){
        ClawServo = hardwareMap.servo.get("clawServo");
    }

    public clawStatuses status;

    public void TeleopControl(boolean open){

        if (open) {
            status = clawStatuses.OPEN;
        } else {
            status = clawStatuses.CLOSED;
        }

        ClawServo.setPosition(status.position());
    }

    public void AutoControl(clawStatuses input) {
        ClawServo.setPosition(input.position());
    }


}
