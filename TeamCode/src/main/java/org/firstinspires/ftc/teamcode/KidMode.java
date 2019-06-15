package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Kid Mode", group = "Rover Ruckus")
public class KidMode extends GenericOpMode {
    private Hardware<KidMode> robot = new Hardware<>(this);

    @Override
    public void runOpMode() {
        double rightValue, leftValue, speedMultiplier = 0.5;

        robot.init(hardwareMap);
        robot.setLiftRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            rightValue = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * speedMultiplier;
            leftValue = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * speedMultiplier;

            robot.setMotorPowers(rightValue, leftValue);

            if (gamepad2.right_trigger > 0)
                robot.getTapeServo().setPower(1);
            else if (gamepad2.left_trigger > 0)
                robot.getTapeServo().setPower(-1);
            else
                robot.getTapeServo().setPower(0);

            if (-0.25 > gamepad2.left_stick_y || gamepad2.left_stick_y > 0.25)
                robot.getTapeControl().setPower(gamepad2.left_stick_y * 0.5);
            else
                robot.getTapeControl().setPower(0);

            telemetry.addData("rightValue", rightValue);
            telemetry.addData("leftValue", leftValue);
            telemetry.addData("rightTrigger", gamepad2.right_trigger);
            telemetry.addData("leftTrigger", gamepad2.left_trigger);
            telemetry.update();
        }
    }
}
