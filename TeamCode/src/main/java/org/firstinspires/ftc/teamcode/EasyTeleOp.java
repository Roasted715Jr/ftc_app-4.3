package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Rover Ruckus", group = "Rover Ruckus")
public class EasyTeleOp extends GenericOpMode {
    private Hardware<EasyTeleOp> robot = new Hardware<>(this);

    @Override
    public void runOpMode() {
        boolean aPressed = false, multiplierToggle = false;
        double rightValue, leftValue, speedMultiplier;

        robot.init(hardwareMap);
        robot.setLiftRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (!aPressed)
                    multiplierToggle = !multiplierToggle;
                aPressed = true;
            } else
                aPressed = false;

            speedMultiplier = multiplierToggle ? 0.5 : 1;

            rightValue = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * speedMultiplier;
            leftValue = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * speedMultiplier;
//            rightValue = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * speedMultiplier;
//            leftValue = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * speedMultiplier;

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

            if (gamepad2.right_bumper)
                robot.setLiftPower(0.75);
            else if (gamepad2.left_bumper)
                robot.setLiftPower(-0.75);
            else
                robot.setLiftPower(0);

            telemetry.addData("rightValue", rightValue);
            telemetry.addData("leftValue", leftValue);
            telemetry.addData("speedMultiplier", speedMultiplier);
            telemetry.addData("rightTrigger", gamepad2.right_trigger);
            telemetry.addData("leftTrigger", gamepad2.left_trigger);
//            telemetry.addData("liftMotor Position", robot.liftMotor.getCurrentPosition()); //-47 to -8700
            telemetry.update();
        }
    }
}
