package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo", group = "CompBot")
@Disabled
public class BasicServoMovement extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.moveServo(0); //This is moving it to dump the marker
        sleep(2000);
        robot.moveServo(1);

        while (opModeIsActive()) {
            telemetry.addData("servoPos", robot.getServo().getPosition());
            telemetry.update();
        }
    }
}
