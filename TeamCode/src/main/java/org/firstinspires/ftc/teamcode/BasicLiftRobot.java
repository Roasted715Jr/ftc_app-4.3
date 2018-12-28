package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Lift Robot", group = "CompBot")
public class BasicLiftRobot extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

//        robot.setLiftPower(1);

//        sleep(1000);

//        robot.liftCounts(535 * 16, 1);

        while (opModeIsActive()) {
            telemetry.addData("Encoder counts", robot.liftMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}
