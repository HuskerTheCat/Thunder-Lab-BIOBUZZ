package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@TeleOp(name = "PEAK SKRIMAGE TELE", group = "Competition")
public class Peak_Scrimage_Tele extends LinearOpMode{

    DcMotor LeftFront = hardwareMap.get(DcMotor.class, "LeftFrontAsDcMotor");
    DcMotor LeftRear = hardwareMap.get(DcMotor.class, "LeftRearAsDcMotor");
    DcMotor RightRear = hardwareMap.get(DcMotor.class, "RightRearAsDcMotor");
    DcMotor RightFront = hardwareMap.get(DcMotor.class, "RightFrontAsDcMotor");
    Servo Turret = hardwareMap.get(Servo.class, "turret");

    IMU imu = hardwareMap.get(IMU.class, "imu");

    ElapsedTime MatchTime;
    double ElapsedTimePeriod;
    @Override
    public void runOpMode() {
        InitializeMotors();
        InitalizeIMU();
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //Launcher.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        //Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.scaleRange(0.2f, 0.8);

        waitForStart();
        while(opModeIsActive()){
            float deadZone = 0.01f;
            //use the joysticks on either controller to move the robot, with priority going to gamepad 1
            //maybe controller priority can be moved to a function with lookup table per interaction?
            RobotCentricDrive((Math.abs(gamepad1.left_stick_y)>deadZone?gamepad1:gamepad2).left_stick_y, //forward/backward
                    (Math.abs(gamepad1.left_stick_x)>deadZone?gamepad1:gamepad2).left_stick_x,//strafe
                    (Math.abs(gamepad1.right_stick_x)>deadZone?gamepad1:gamepad2).right_stick_x);//rotation

            //basic turret rotation with priority on controller 1
            //if both are pressed just stops, eventualy will go with whichever bumper was pressed most recently
            if(gamepad2.right_bumper&&!gamepad2.left_bumper){
                Turret.setPosition(Turret.getPosition()+0.01);
            } else if (gamepad2.left_bumper&&!gamepad2.right_bumper) {
                Turret.setPosition(Turret.getPosition()-0.01);
            } else if (gamepad1.right_bumper&&!gamepad1.left_bumper) {
                Turret.setPosition(Turret.getPosition()+0.01);
            } else if (gamepad1.left_bumper&&!gamepad1.right_bumper) {
                Turret.setPosition(Turret.getPosition()-0.01);
            }


        }
    }

    private void RobotCentricDrive(double RobotForward, double RobotRight, double RobotClockwise) {

        // ---Copy to any OpMode with Mecanum Drive for Robot-centric Control---
        telemetry.addData("Drive Mode", "Robot Centric");
        telemetry.addData("Robot Forward", Double.parseDouble(JavaUtil.formatNumber(RobotForward, 2)));
        telemetry.addData("Robot Right", Double.parseDouble(JavaUtil.formatNumber(RobotRight, 2)));
        telemetry.addData("Robot Clockwise", Double.parseDouble(JavaUtil.formatNumber(RobotClockwise, 2)));
        MecanumDrive(RobotForward, RobotRight, RobotClockwise);
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
    private void InitializeMotors() {
        // Set Drive Motors to Brake on Zero Power
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set Launcher Motor thr Brake on Zero power
        // Set  Intake & Launcher Motor Directions
        // Set Drive Motor Directions
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        // Set Servo Directions
        // Motor encoder
    }
    void InitalizeIMU(){

    }
}
