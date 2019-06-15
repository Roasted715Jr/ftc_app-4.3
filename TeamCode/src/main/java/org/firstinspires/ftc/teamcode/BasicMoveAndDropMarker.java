package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Drop Marker", group = "Auton Tests")
@Disabled
public class BasicMoveAndDropMarker extends GenericOpMode {
    private static final int TAPE_NOT_FOUND = 0;
    private static final int TAPE_COLOR_BLUE = 1;
    private static final int TAPE_COLOR_RED = 2;
//    private static final int[] MIN_BLUE = {15, 30, 40};
    private static final int[] MIN_BLUE = {15, 30, 40};
//    private static final int[] MAX_BLUE = {35, 50, 55};
    private static final int[] MAX_BLUE = {35, 50, 55};
//    private static final int[] MIN_RED = {80, 30, 30};
    private static final int[] MIN_RED = {1500, 30, 30};
//    private static final int[] MAX_RED = {90, 50, 45};
    private static final int[] MAX_RED = {1700, 50, 45};

    private Hardware<BasicMoveAndDropMarker> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        int[] color = new int[3];
        int tapeColor = TAPE_NOT_FOUND;

        waitForStart();

        robot.setMotorPowers(1);

        while (opModeIsActive()) {
            color[0] = robot.leftSensor.red();
            color[1] = robot.leftSensor.green();
            color[2] = robot.leftSensor.blue();

            if (isBetween(color, MIN_BLUE, MAX_BLUE))
                tapeColor = TAPE_COLOR_BLUE;
            else if (isBetween(color, MIN_RED, MAX_RED))
                tapeColor = TAPE_COLOR_RED;
            else
                tapeColor = TAPE_NOT_FOUND;

            if (tapeColor != TAPE_NOT_FOUND)
                robot.setMotorPowers(0);

            telemetry.addData("Tape Color", tapeColor);
            telemetry.update();
        }
    }

    private boolean isBetween(int[] val, int[] min, int[] max) {
        for (int i = 0; i < val.length; i++) {
            if (!(min[i] <= val[i] && val[i] <= max[i])) {
                return false;
            }
        }

        return true;
    }
}
