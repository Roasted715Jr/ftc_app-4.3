package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo", group = "CompBot")
@Disabled
public class BasicServoMovement extends GenericOpMode {
    private Hardware<BasicServoMovement> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.moveServo(0); //This is moving it to dump the marker
        sleep(2000);
        robot.moveServo(1);

        while (opModeIsActive()) {
            telemetry.addData("servoPos", robot.getBaseServo().getPosition());
            telemetry.update();
        }
    }
}
