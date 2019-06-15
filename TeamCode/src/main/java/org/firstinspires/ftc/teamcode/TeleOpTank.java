package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Drive", group = "Rover Ruckus")
public class TeleOpTank extends GenericOpMode {
    private Hardware<TeleOpTank> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        boolean aPressed = false, multiplierToggle = false;
        double rightValue, leftValue, speedMultiplier = 1;

        robot.init(hardwareMap);
        robot.setLiftRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            rightValue = -gamepad1.right_stick_y * speedMultiplier;
            leftValue = -gamepad1.left_stick_y * speedMultiplier;

            robot.setMotorPowers(rightValue, leftValue);

            if (gamepad1.a) {
                if (!aPressed)
                    multiplierToggle = !multiplierToggle;
                aPressed = true;
            } else
                aPressed = false;

            speedMultiplier = multiplierToggle ? 0.5 : 1;

            if (gamepad2.right_bumper) {
//                telemetry.addData("Status", "Lifting robot");
//                telemetry.update();
                robot.setLiftPower(0.75);
//                robot.setMotorPowers(0);
//                robot.liftExtendPartial();
//                robot.liftRetract();
            } else if (gamepad2.left_bumper) {
                robot.setLiftPower(-0.75);
            } else
                robot.setLiftPower(0);

            telemetry.addData("leftValue", "%.2f", leftValue);
            telemetry.addData("rightValue", "%.2f", rightValue);
            telemetry.addData("speedMultiplier", speedMultiplier);
//            telemetry.addData("liftMotor Position", robot.liftMotor.getCurrentPosition()); //-47 to -8700
            telemetry.update();
        }
    }
}
