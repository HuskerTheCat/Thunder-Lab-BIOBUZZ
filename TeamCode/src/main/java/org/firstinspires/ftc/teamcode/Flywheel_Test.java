package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Max Test", group = "Test")
public class Flywheel_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx fw = hardwareMap.get(DcMotorEx.class, "Flywheel");
        fw.setDirection(DcMotorSimple.Direction.REVERSE);
        fw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double max = 0;
        waitForStart();
        fw.setPower(1.0);
        while (opModeIsActive()) {
            max = Math.max(max, fw.getVelocity());
            telemetry.addData("velocity", fw.getVelocity());
            telemetry.addData("max seen", max);
            telemetry.addData("SDK max ticks/s", fw.getMotorType().getAchieveableMaxTicksPerSecond());
            telemetry.update();
        }
    }
}