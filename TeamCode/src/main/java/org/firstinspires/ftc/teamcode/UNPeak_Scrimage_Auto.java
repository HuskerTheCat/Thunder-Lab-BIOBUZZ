package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

//name has to be different from Peak_Scrimage_Auto or the robot app reports a duplicate
@Autonomous(name = "UNPeak Scrimage Auto", group = "Auto",preselectTeleOp = "PEAK SCRIMMAGE TELE")
public class UNPeak_Scrimage_Auto extends LinearOpMode {
    DcMotor LeftFront;
    DcMotor LeftRear;
    DcMotor RightRear;
    DcMotor RightFront;
    DcMotor Turret;
    Servo FlowerIntake;
    DcMotorEx Flywheel;
    DcMotor Intake;
    IMU imu;
    final double FlowerIntakeServoRangeDeg = 270;
    final double FlowerIntakeUpPos = 0 / FlowerIntakeServoRangeDeg;
    final double FlowerIntakeDownPos = 120 / FlowerIntakeServoRangeDeg;
    final double flywheelVelocity = 1000; //encoder ticks per second
    final double TurretPower = 0.4;
    final double IntakePower = 0.8;

    //about one wheel turn on a goBILDA 312rpm motor (537.7 ticks/rev), tune on the robot
    final int DriveDistanceTicks = 540;

    boolean blueAlliance = true;
    public void runOpMode(){
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        Turret = hardwareMap.get(DcMotor.class, "Turret");
        FlowerIntake = hardwareMap.get(Servo.class, "FlowerIntake");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        //FeedTop = hardwareMap.get(CRServo.class, "TopFeed");
        //FeedBottom = hardwareMap.get(CRServo.class, "BottomFeed");
        //LaunchServo = hardwareMap.get(Servo.class, "launcherServo");
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // same directions as Peak_Scrimage_Tele
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.REVERSE); //check on robot, usually both right motors are FORWARD
        while(opModeInInit()){

            telemetry.addLine("Select with dpad");
            blueAlliance = (gamepad1.dpad_left||gamepad2.dpad_left) || (blueAlliance && !(gamepad1.dpad_right||gamepad2.dpad_right));

            telemetry.addLine("Alliance: " + (blueAlliance ?"[Blue]":" Blue ") + " | " +(!blueAlliance ?"[Red]":" Red "));
            telemetry.update();
        }
        waitForStart();
        double pos = LeftFront.getCurrentPosition();

        //abs so it still stops if the encoder counts the other way
        while(Math.abs(LeftFront.getCurrentPosition()-pos)<DriveDistanceTicks&&opModeIsActive()){
            MecanumDrive(0.5, 0.5, 0);
        }
        MecanumDrive(0,0,0);


    }
    private void MecanumDrive(double RobotForward, double RobotRight, double RobotClockwise) {
        /*
         * This function takes power setting inputs for the three robot-centric drive commands
         * and converts them into motor power commands for a Mecanum wheeled robot. It wheels are
         * turning the wrong direction, switch their direction in the InitializeMotors function.
         */
        double MaxInput;

        // ---Copy to any OpMode for Robot-centric Mecanum Wheel Drive---
        // Scale to Avoid Clipping
        // Get the sum of the magnitudes of the robot commands. One of the motor commands will equal this maximum input.
        MaxInput = Math.abs(RobotForward) + Math.abs(RobotRight) + Math.abs(RobotClockwise);
        // If the maximum input is greater than 1, the motor will clip anything greater than 1 when applying power.
        if (MaxInput > 1) {
            // Divide the current input by the maximum input to scale it relative to the other inputs.
            RobotForward = RobotForward / MaxInput;
            // Divide the current input by the maximum input to scale it relative to the other inputs.
            RobotRight = RobotRight / MaxInput;
            // Divide the current input by the maximum input to scale it relative to the other inputs.
            RobotClockwise = RobotClockwise / MaxInput;
        }
        // Transform Robot Commands to Mecanum Motors
        // Left Front Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        LeftFront.setPower(RobotForward + RobotRight + RobotClockwise);
        // Left Rear Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        LeftRear.setPower((RobotForward - RobotRight) + RobotClockwise);
        // Right Front Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        RightFront.setPower((RobotForward - RobotRight) - RobotClockwise);
        // Right Rear Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        RightRear.setPower((RobotForward + RobotRight) - RobotClockwise);
    }
}

