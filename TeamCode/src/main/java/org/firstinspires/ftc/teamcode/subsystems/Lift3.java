package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 * don't use unless we really have to
 */
@Config
public class Lift3 {
    public enum liftLevels {
        FLOOR (0),
        LOW (0),
        MED (9),
        HIGH (20);
        int height;
        liftLevels(int height) {
            this.height = height;
        }
    }
    private static final double TICKS_PER_REV = 532;
    public static int TeleopTopAdjust = -2;

    public static double SPOOL_RADIUS = 0.70; // in
    public static double GEAR_RATIO = 1; // output (spool) speed / input (motor) speed

    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]
    public static double MAX_HEIGHT = 24; // in
    private boolean running = false;


    public static double MAX_VEL = 25; // in/s
    public static double MAX_ACCEL = 30; // in/s^2
    public static double MAX_JERK = 22; // in/s^3
    public static double kV = 1 / rpmToVelocity(getMaxRpm());
    public static double kA = 0;
    public static double kStatic = 0;

    public static double p = 0.002;
    public  static  double i = 0;
    public  static  double d = 0;



    public static double power_modifier = 0.4;


    public DcMotor leftLiftMotor;
    public DcMotor rightLiftMotor;
  //  private PIDController controller;
    private double desiredHeight = 0;
    private int offset;
    private Gamepad gamepad2;

    public static PIDController controller = new PIDController(p, i, d);

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    private static int inchesToEncoderTicks(double inches) {
        return (int) ((inches / (SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO)) * TICKS_PER_REV);
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 316;
    }


    private  Telemetry telemetry;
    public Lift3(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp) {
        leftLiftMotor = hardwareMap.dcMotor.get( "leftLiftMotor");
        rightLiftMotor = hardwareMap.dcMotor.get("rightLiftMotor");
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gamepad2 = gp;
        // if necessary, reverse the motor so "up" is positive
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setTolerance(80);
        // motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward;
        offset = rightLiftMotor.getCurrentPosition();
        desiredHeight = offset;

        //set telemetry
        this.telemetry = telemetry;
    }

    public boolean isBusy() {
        return !controller.atSetPoint();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);


        this.desiredHeight = inchesToEncoderTicks(height);
        controller.setSetPoint(desiredHeight);
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(rightLiftMotor.getCurrentPosition() - offset);
    }

    public void update() {
        double power;
        double currentHeight = getCurrentHeight();
        int state = rightLiftMotor.getCurrentPosition();
        power = controller.calculate(state, desiredHeight);

        telemetry.addData("currentHeight", currentHeight);
        telemetry.addData("pos err", controller.getPositionError());
        telemetry.update();
        setPower(power);
    }

    public void setPower(double power) {
        this.telemetry.addData("power", power);
        if (power < 0 && rightLiftMotor.getCurrentPosition() <= offset && !gamepad2.right_bumper) {
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);

        } else {
            leftLiftMotor.setPower(power);
            rightLiftMotor.setPower(power);

        }

    }

    public void teleOpControl() {
        if (gamepad2.dpad_down) {
            setHeight(liftLevels.LOW.height);

        } else if (gamepad2.dpad_left) {
            setHeight(liftLevels.LOW.height);
        } else if (gamepad2.dpad_up) {
            setHeight(liftLevels.MED.height);
        } else if (gamepad2.dpad_right) {
            setHeight(liftLevels.HIGH.height+TeleopTopAdjust);
        }
        if (isBusy()) {
            update();
        } else {
                setPower(-gamepad2.right_stick_y*power_modifier);
                desiredHeight = rightLiftMotor.getCurrentPosition();
            }
        telemetry.addData("currentHeight", getCurrentHeight());

    }

    public void setAutoPosition(liftLevels level) {
        setHeight(level.height);
    }

}