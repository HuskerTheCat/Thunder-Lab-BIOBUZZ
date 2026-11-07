package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.math.Pose;

@Autonomous(name = "Peak Scrimage Auto", group = "Auto",preselectTeleOp = "PEAK SCRIMMAGE TELE")
public class Peak_Scrimage_Auto extends LinearOpMode {
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

    boolean blueAlliance = true;
    public void runOpMode(){
        while(opModeInInit()){
            telemetry.addLine("Select with dpad");
            blueAlliance = (gamepad1.dpad_left||gamepad2.dpad_left) || (blueAlliance && !(gamepad1.dpad_right||gamepad2.dpad_right));

            telemetry.addLine("Alliance: " + (blueAlliance ?"[Blue]":" Blue ") + " | " +(!blueAlliance ?"[Red]":" Red "));
            telemetry.update();
        }
        waitForStart();
        final Pose[] poses = {
                new Pose(1,1),
                new Pose(2,2)};





    }

}
