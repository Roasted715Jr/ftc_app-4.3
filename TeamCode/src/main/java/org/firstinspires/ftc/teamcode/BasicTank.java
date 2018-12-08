package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tank", group = "CompBot")
//@Disabled
public class BasicTank extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        double rightValue;
        double leftValue;

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rightValue = -gamepad1.right_stick_y;
            leftValue = -gamepad1.left_stick_y;

            robot.setMotorPowers(rightValue, leftValue);

            telemetry.addData("leftValue", "%.2f", leftValue);
            telemetry.addData("rightValue", "%.2f", rightValue);

            telemetry.update();

            //Run this 25 times/second
            sleep(40);
        }
    }
}
