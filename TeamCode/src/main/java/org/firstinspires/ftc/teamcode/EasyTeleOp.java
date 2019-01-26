package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Easy TeleOp", group = "CompBot")
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

            robot.setMotorPowers(rightValue, leftValue);

            

            if (gamepad2.right_bumper)
                robot.setLiftPower(0.75);
            else if (gamepad2.left_bumper)
                robot.setLiftPower(-0.75);
            else
                robot.setLiftPower(0);

            telemetry.addData("rightValue", rightValue);
            telemetry.addData("leftValue", leftValue);
            telemetry.addData("speedMultiplier", speedMultiplier);
//            telemetry.addData("liftMotor Position", robot.liftMotor.getCurrentPosition()); //-47 to -8700
            telemetry.update();
        }
    }
}
