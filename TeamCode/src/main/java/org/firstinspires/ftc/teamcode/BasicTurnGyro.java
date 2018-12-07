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
    private static final int TAPE_NOT_FOUND = 0;
    private static final int TAPE_COLOR_BLUE = 1;
    private static final int TAPE_COLOR_RED = 2;
    private static final int[] MIN_BLUE = {15, 30, 40};
    private static final int[] MAX_BLUE = {35, 50, 55};
    private static final int[] MIN_RED = {80, 30, 30};
    private static final int[] MAX_RED = {90, 50, 45};

    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        int deg = 22;
        double speed = 0.25;
//        robot.turnDegrees(deg, speed);
        robot.turnToDegree(deg, speed);

//        robot.goDistance(36, 1);
//        robot.turnToDegree(deg, speed);

//        dropIdol();

        while (opModeIsActive()) {
            telemetry.addData("yaw", robot.getYaw());
            telemetry.addData("Amount past", Math.abs(robot.getYaw() - deg));
            telemetry.update();
            idle();
        }
    }

    private void dropIdol() {
        int[] color = new int[3];
        int tapeColor = TAPE_NOT_FOUND;

        robot.setMotorPowers(0.5);

        while (tapeColor == TAPE_NOT_FOUND) {
            color[0] = robot.getColorSensor().red();
            color[1] = robot.getColorSensor().green();
            color[2] = robot.getColorSensor().blue();

            if (isBetween(color, MIN_BLUE, MAX_BLUE))
                tapeColor = TAPE_COLOR_BLUE;
            else if (isBetween(color, MIN_RED, MAX_RED))
                tapeColor = TAPE_COLOR_RED;
            else
                tapeColor = TAPE_NOT_FOUND;
        }

        robot.setMotorPowers(0);

        robot.moveServo(0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.moveServo(1);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private boolean isBetween(int[] val, int[] min, int[] max) {
        boolean isBetween = true;

        for (int i = 0; i < val.length; i++) {
            if (!(min[i] <= val[i] && val[i] <= max[i])) {
                isBetween = false;
                break;
            }
        }

        return isBetween;
    }
}
