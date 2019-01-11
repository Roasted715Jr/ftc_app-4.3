package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

class AutonProcedures<T extends RunningOpMode> {
    private static final int BLOCK_NOT_FOUND = 0;
    private static final int RIGHT_POSITION = 1;
    private static final int CENTER_POSITION = 2;
    private static final int LEFT_POSITION = 3;
    private static final int DEAD_CENTER = 4;

//    private static final int[] MIN_BLUE = {15, 30, 40};
//    private static final int[] MAX_BLUE = {35, 50, 55};
//    private static final int[] MIN_RED = {80, 30, 30};
//    private static final int[] MAX_RED = {90, 50, 45};

    private static final int IMG_CUTOFF_Y = 300; //115
    private static final int IMG_FIRST_SECTION_X = 110;
    private static final int IMG_THIRD_SECTION_X = 290;
    private static final double FIND_BLOCK_SPEED = 0.2;

    static final int CRATER_START = 0;
    static final int DEPOT_START = 0;

//    private static final int ANGLE_TOLERANCE = 40;
    private static final double TURN_SPEED = 0.25;

//    private double startYaw, endYaw;
    private ElapsedTime elapsedTime;
//    enum StartPosition {
//        CRATER,
//        DEPOT
//    }
    private GoldVision goldVision;
    private Hardware robot;
    private HardwareMap hardwareMap;
    private int cCounter, dcCounter, lCounter, rCounter, blockPos = BLOCK_NOT_FOUND, blockPosRel = BLOCK_NOT_FOUND, degToTurn = 22, blockDist, startPosition;
//    private StartPosition startPosition;
    private T runningOpMode;
    private VuforiaLocalizer vuforia;

    void init(ElapsedTime elapsedTime, Hardware robot, HardwareMap hardwareMap, int startPosition, T runningOpMode) {
        this.elapsedTime = elapsedTime;
        this.robot = robot;
        this.hardwareMap = hardwareMap;
        this.startPosition = startPosition;
        this.runningOpMode = runningOpMode;

        robot.init(hardwareMap);
        robot.initCamera();
        goldVision = new GoldVision();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = Hardware.VUFORIA_LICENSE_KEY;
        parameters.cameraName = robot.getWebcam();
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();
    }

    private void displayTelemetry(String msg) {
        runningOpMode.displayTelemetry(msg);
    }

    void start() {
        displayTelemetry("Deploying");
        deploy();
        displayTelemetry("Going to block");
        goToBlock();
        displayTelemetry("Going to depot");
        goToDepot();
        displayTelemetry("Parking");
        park();
        displayTelemetry("Done");
    }

    private void deploy() {
        robot.liftReset();
        robot.liftExtendFull(elapsedTime);
        //We have the retract code after we start moving to make sure we don't start retracting before we move (it is in the goToBlock function now)
    }

    private void goToBlock() {
        blockPos = getBlockPos(1000);
//        blockPos = CENTER_POSITION;

        if (blockPos == RIGHT_POSITION) {
            degToTurn = -20;
            blockDist = 32;
        } else if (blockPos == LEFT_POSITION) {
            degToTurn = 20;
            blockDist = 32;
        } else if (blockPos == CENTER_POSITION) {
            blockDist = 30;
        }

        displayTelemetry("blockPos: " + blockPos);

        robot.goDistance(3, 1);
        new Thread(new Runnable() {
            @Override
            public void run() {
                robot.liftRetract();
            }
        }).start();
        if (blockPos != CENTER_POSITION)
//            robot.turnDegrees(degToTurn, TURN_SPEED);
            displayTelemetry("If we see this, it is not a good sign");
        else {
            displayTelemetry("Good job");
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        robot.goDistance(blockDist, 1);
    }

    private int getBlockPos(int mSec) {
        int blockPos = BLOCK_NOT_FOUND;

        lCounter = 0;
        cCounter = 0;
//        dcCounter = 0;
        rCounter = 0;

        elapsedTime.reset();

        while (elapsedTime.milliseconds() <= mSec)
            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
                @Override
                public void accept(Frame frame) {
                    Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                    Mat img = new Mat();
                    if (bitmap != null) {
                        Utils.bitmapToMat(bitmap, img);
                    }

                    goldVision.processFrame(img, null);

                    List<MatOfPoint> contours = goldVision.getContours();

                    for (int i = 0; i < contours.size(); i++) {
                        // get the bounding rectangle of a single contour, we use it to get the x/y center
                        // yes there's a mass center using Imgproc.moments but w/e
                        Rect boundingRect = Imgproc.boundingRect(contours.get(i));

                        double boundingRectX = (boundingRect.x + boundingRect.width) / 2;
                        double boundingRectY = (boundingRect.y + boundingRect.height) / 2;

                        if (boundingRectY <= IMG_CUTOFF_Y) {
                            if (boundingRectX < IMG_FIRST_SECTION_X - 10)
                                rCounter++; //These are reversed now that the camera will be upside down
//                            else if (boundingRectX > IMG_FIRST_SECTION_X + 75 && boundingRectX < IMG_THIRD_SECTION_X - 75)
//                                dcCounter++;
                            else if (boundingRectX > IMG_FIRST_SECTION_X + 10 && boundingRectX < IMG_THIRD_SECTION_X - 10)
                                cCounter++;
                            else if (boundingRectX > IMG_THIRD_SECTION_X + 10)
                                lCounter++;

//                            telemetry.addData("Coordinates", "(" + boundingRectX + ", " + boundingRectY + ")");
                        }
                    }
                }
            }));

        if (rCounter > cCounter)
            if (rCounter > lCounter)
                blockPos = RIGHT_POSITION;
            else
                blockPos = LEFT_POSITION;
        else if (lCounter > cCounter)
            blockPos = LEFT_POSITION;
//        else if (dcCounter > cCounter)
//            blockPos = DEAD_CENTER;
        else if (cCounter > 0)
            blockPos = CENTER_POSITION;

//        telemetry.addData("blockPos1", blockPos);
//        telemetry.update();

        return blockPos;
    }

    private void goToDepot() {
        if (startPosition == CRATER_START) {
            //Move away from the block
//            robot.goDistance(-blockDist + 5, 1);
            robot.goDistance(-blockDist + 20, 1);
//            robot.turnToDegree(45, TURN_SPEED);
            robot.turnToDegree(90, TURN_SPEED);
            robot.goDistance(36, 1);
//            robot.turnDegrees(55, TURN_SPEED);
//            robot.turnDegrees(10, TURN_SPEED, TURN_SPEED * 2);
        } else
            if (blockPos != CENTER_POSITION)
                robot.turnDegrees(degToTurn * -2, TURN_SPEED);

//        robot.goDistance(9, 1);
        dropIdol();
    }

    private void dropIdol() {
        int red;
        int blue;
        float[] hsvValues = new float[3];
        float hue;

        robot.setMotorPowers(0.75);

        while (true) {
            red = robot.getColorSensor().red();
            blue = robot.getColorSensor().blue();

            Color.RGBToHSV(robot.getColorSensor().red(),
                     robot.getColorSensor().green(),
                    robot.getColorSensor().blue(),
                    hsvValues);
            hue = hsvValues[0];

            if (red > 1000 && (hue < 10 || hue > 350))
                break;
            else if (blue > 1000 && (hue < 230 && hue > 190))
                break;
        }

        if (startPosition == DEPOT_START)
            robot.goDistance(12, 0.5);

        robot.setMotorPowers(0);

        robot.moveServo(0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.moveServo(1);
    }

//    private boolean isBetween(int[] val, int[] min, int[] max) {
//        boolean isBetween = true;
//
//        for (int i = 0; i < val.length; i++) {
//            if (!(min[i] <= val[i] && val[i] <= max[i])) {
//                isBetween = false;
//                break;
//            }
//        }
//
//        return isBetween;
//    }

    private void park() {
        if (startPosition == CRATER_START)
            robot.goDistance(-77, 1);
        else {
            robot.turnToDegree(-60, TURN_SPEED);
            robot.goDistance(-20, 1);
            robot.turnToDegree(-50, TURN_SPEED);
            robot.goDistance(-57, 1);
        }

        robot.setMotorPowers(0);
    }
}
