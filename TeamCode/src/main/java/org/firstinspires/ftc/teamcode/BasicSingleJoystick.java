package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Single Joystick", group = "CompBot")
@Disabled
public class BasicSingleJoystick extends LinearOpMode {
    private static final int DEG_TOLERANCE = 5;

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            double xValue = gamepad1.left_stick_x;
            double yValue = -gamepad1.left_stick_y;
            double rightPower = 0;
            double leftPower = 0;
            double angle;

            angle = Math.atan(yValue / xValue) * 180 / Math.PI;

            if (xValue < 0)
                angle += 180;
            else
                if (yValue < 0)
                    angle += 360;

            if (!Double.isNaN(angle)) {
                if (angle == 0) {
                    rightPower = -1;
                    leftPower = 1;
                } else if (angle > 0 && angle < 180) {
                    if (angle < 90) {
                        rightPower = (angle - 45) / 45;
                        leftPower = 1;
                    } else if (angle == 90) {
                        rightPower = 1;
                        leftPower = 1;
                    } else {
                        rightPower = 1;
                        leftPower = -(angle - 135) / 45;
                    }
                } else if (angle == 180) {
                    rightPower = 1;
                    leftPower = -1;
                } else {
                    if (angle < 270) {
                        rightPower = -(angle - 2255) / 45;
                        leftPower = -1;
                    } else if (angle == 270) {
                        rightPower = -1;
                        leftPower = -1;
                    } else {
                        rightPower = -1;
                        leftPower = (angle - 315) / 45;
                    }
                }
            }

            robot.setMotorPowers(rightPower, leftPower);

            telemetry.addData("angle", "%.2f degrees", angle);
            telemetry.update();
        }
    }
}
