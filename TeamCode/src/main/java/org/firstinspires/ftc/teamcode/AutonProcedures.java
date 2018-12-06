package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.DcMotor;
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

class AutonProcedures {
    private static final int BLOCK_NOT_FOUND = 0;
    private static final int RIGHT_POSITION = 1;
    private static final int CENTER_POSITION = 2;
    private static final int LEFT_POSITION = 3;
    private static final int DEAD_CENTER = 4;

    private static final int TAPE_NOT_FOUND = 0;
    private static final int TAPE_COLOR_BLUE = 1;
    private static final int TAPE_COLOR_RED = 2;
    private static final int[] MIN_BLUE = {15, 30, 40};
    private static final int[] MAX_BLUE = {35, 50, 55};
    private static final int[] MIN_RED = {80, 30, 30};
    private static final int[] MAX_RED = {90, 50, 45};

    private static final int IMG_CUTOFF_Y = 300; //115
    private static final int IMG_FIRST_SECTION_X = 110;
    private static final int IMG_THIRD_SECTION_X = 290;

    private static final int ANGLE_TOLERANCE = 40;

    private double startYaw, endYaw;
    private ElapsedTime elapsedTime;
    enum StartPosition {
        CRATER,
        DEPOT
    }
    private GoldVision goldVision;
    private Hardware robot;
    private HardwareMap hardwareMap;
    private int cCounter, dcCounter, lCounter, rCounter, blockPos = BLOCK_NOT_FOUND;
    private VuforiaLocalizer vuforia;

    void init(ElapsedTime elapsedTime, Hardware robot, HardwareMap hardwareMap) {
        this.elapsedTime = elapsedTime;
        this.robot = robot;
        this.hardwareMap = hardwareMap;

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

    void start(StartPosition startPosition) {
        deploy();
        goToBlock();
        if (startPosition == StartPosition.CRATER)
            goToDepot();
        dropIdol();
        park();
    }

    private void deploy() {}

    private void goToBlock() {
        blockPos = getBlockPos(1000);

        if (blockPos == RIGHT_POSITION || blockPos == LEFT_POSITION) {
            turnToBlock();
            robot.goDistance(36, 1);
        } else if (blockPos == DEAD_CENTER)
            robot.goDistance(30, 1); //65, test out 30
    }

    private int getBlockPos(int mSec) {
        int blockPos = 0;

        lCounter = 0;
        cCounter = 0;
        dcCounter = 0;
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
                            else if (boundingRectX > IMG_FIRST_SECTION_X + 75 && boundingRectX < IMG_THIRD_SECTION_X - 75)
                                dcCounter++;
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
        else if (dcCounter > cCounter)
            blockPos = DEAD_CENTER;
        else if (cCounter > 0)
            blockPos = CENTER_POSITION;

//        telemetry.addData("blockPos1", blockPos);
//        telemetry.update();

        return blockPos;
    }

    private void turnToBlock() {
        if (blockPos != BLOCK_NOT_FOUND) {
            startYaw = robot.getYaw();

            turn:
            while (getBlockPos(100) != DEAD_CENTER) {
                //Move motors then stop them after a short amount of time
                switch (blockPos) {
                    case RIGHT_POSITION:
                        if (robot.getYaw() - startYaw < -ANGLE_TOLERANCE)
                            while (robot.getYaw() < -ANGLE_TOLERANCE / 2)
                                robot.setMotorPowers(1, -1);
                        else
                            robot.setMotorPowers(-0.25, 0.25);
                        break;
                    case LEFT_POSITION:
                        if (robot.getYaw() - startYaw > ANGLE_TOLERANCE)
                            while (robot.getYaw() > ANGLE_TOLERANCE / 2)
                                robot.setMotorPowers(-1, 1);
                        robot.setMotorPowers(0.25, -0.25);
                        break;
                    default:
                        break turn;
                }
                try {
                    Thread.sleep(125);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.setMotorPowers(0, 0);
            }
        }

        endYaw = robot.getYaw();
    }

//    private void awayFromBlock() {
//        if (blockPos == RIGHT_POSITION || blockPos == LEFT_POSITION)
//            robot.goDistance(-30, 1);
//        else if (blockPos == DEAD_CENTER)
//            robot.goDistance(-60, 1);
//    }

    private void goToDepot() {}

    private void dropIdol() {
        int[] color = new int[3];
        int tapeColor = TAPE_NOT_FOUND;
        while (!isBetween(color, MIN_BLUE, MAX_BLUE) && !isBetween(color, MIN_RED, MAX_RED)) {
            color[0] = robot.getColorSensor().red();
            color[1] = robot.getColorSensor().green();
            color[2] = robot.getColorSensor().blue();
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

    private void park() {}
}
