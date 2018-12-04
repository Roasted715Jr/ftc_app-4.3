package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@TeleOp(name = "Turn w/ Gyro", group = "CompBot")
//@Disabled
public class BasicTurnGyro extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        int deg = 90;
        double speed = 1;
        robot.turnDegrees(deg, speed);

        while (opModeIsActive()) {
            telemetry.addData("yaw", robot.getYaw());
            telemetry.addData("Amount past", Math.abs(robot.getYaw() - deg));
            telemetry.update();
            idle();
        }
    }
}
