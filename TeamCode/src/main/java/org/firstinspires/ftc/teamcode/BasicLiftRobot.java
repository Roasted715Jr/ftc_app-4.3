package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Lift Robot", group = "Auton Tests")
@Disabled
public class BasicLiftRobot extends GenericOpMode {
    Hardware<BasicLiftRobot> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

//        robot.setLiftPower(1);

//        sleep(1000);

//        robot.liftToPos(535 * 16);

        while (opModeIsActive()) {
            telemetry.addData("Encoder counts", robot.liftMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}
