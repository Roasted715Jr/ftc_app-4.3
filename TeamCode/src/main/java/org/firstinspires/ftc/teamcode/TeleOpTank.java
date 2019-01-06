package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank", group = "CompBot")
public class TeleOpTank extends LinearOpMode {
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

            if (gamepad2.right_bumper) {
                telemetry.addData("Status", "Lifting robot");
                telemetry.update();
                robot.liftExtendPartial();
                robot.liftRetract();
            }

            if (gamepad2.a) {
                robot.setLiftRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.setLiftPower(-0.5);
            } else
                robot.setLiftPower(0);

            if (gamepad2.left_bumper)
                robot.liftReset();

            //Run this 25 times/second
            sleep(40);
        }
    }
}
