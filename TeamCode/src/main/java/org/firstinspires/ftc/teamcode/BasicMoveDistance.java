package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Tutorial", group = "CompBot")
//@Disabled
public class BasicMoveDistance extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.goDistance(12, 1);

        while (robot.rightMotor.isBusy() && robot.leftMotor.isBusy()) { //Use an && so we don't have one motor overshoot
            telemetry.addData("Wheel Circumference", robot.getWheelCircumferenceInch());
            telemetry.addData("Revolutions", 12 / (robot.getWheelCircumferenceInch() * 35 / 45));
            telemetry.addData("Target Counts", (int) (12 / (robot.getWheelCircumferenceInch() * 35 / 45) * 1120));
            telemetry.addData("Right Encoder", robot.rightMotor.getCurrentPosition());
            telemetry.addData("Left Encoder", robot.leftMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.setMotorPowers(0);

        //Much appreciated code from a user
//        DcMotor motor = ...;
//        int offset = motor.getCurrentPosition();
//        int rampTicks = 2000;
//        while (opModeIsActive()) {
//            int pos = motor.getCurrentPosition() - offset;
//            motor.setPower(pos / (double) rampTicks);
//        }

        while (opModeIsActive()) {
            telemetry.addData("Wheel Circumference", robot.getWheelCircumferenceInch());
            telemetry.addData("Revolutions", 12 / (robot.getWheelCircumferenceInch() * 35 / 45));
            telemetry.addData("Target Counts", (int) (12 / (robot.getWheelCircumferenceInch() * 35 / 45) * 1120));
            telemetry.addData("Right Encoder", robot.rightMotor.getCurrentPosition());
            telemetry.addData("Left Encoder", robot.leftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
