package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "Webcam Test", group = "CompBot")
@Disabled
public class BasicWebcam extends LinearOpMode {
    private static final int BLOCK_NOT_FOUND = 0;
    private static final int RIGHT_POSITION = 1;
    private static final int CENTER_POSITION = 2;
    private static final int LEFT_POSITION = 3;
    private static final int DEAD_CENTER = 4;

    private static final int IMG_CUTOFF_Y = 300; //115
    private static final int IMG_FIRST_SECTION_X = 110;
    private static final int IMG_THIRD_SECTION_X = 290;

    private static final int ANGLE_TOLERANCE = 40;

    private double startYaw, endYaw;
    private GoldVision goldVision;
    private Hardware robot = new Hardware();
    private int cCounter, dcCounter, lCounter, rCounter, blockPos = BLOCK_NOT_FOUND;
    private VuforiaLocalizer vuforia;

    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        goldVision = new GoldVision();
        robot.init(hardwareMap);
        robot.initCamera();

//        rCounter = 0;
//        dcCounter = 0;
//        cCounter = 0;
//        lCounter = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = Hardware.VUFORIA_LICENSE_KEY;
        parameters.cameraName = robot.getWebcam();

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

//        waitForStart();
		//This code will replace the waitForStart function
		while (!opModeIsActive() && !isStopRequested()) {
			telemetry.addData("Status", "Waiting in init");
			telemetry.update();
		}

        telemetry.clearAll();

        blockPos = getBlockPos(1000);
        telemetry.addData("blockPos", blockPos);
        telemetry.update();

        turnToBlock();

//        goToBlock();

//        awayFromBlock();

        while (opModeIsActive()) {
            idle();
        }
    }

    private void goToBlock () {
        cCounter = 0;
        dcCounter = 0;
        lCounter = 0;
        rCounter = 0;

        blockPos = getBlockPos(1000);

        if (blockPos == RIGHT_POSITION || blockPos == LEFT_POSITION) {
            turnToBlock();
            robot.goDistance(36, 1);
        } else if (blockPos == DEAD_CENTER)
            robot.goDistance(65, 1);
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

//    private void turnToBlock() {
//        telemetry.addData("Func Start", "turnToBlock");
//        telemetry.update();
//        if (blockPos != BLOCK_NOT_FOUND) {
//            turn:
//            while (getBlockPos(100) != DEAD_CENTER) {
//                //Move motors then stop them after a short amount of time
//                switch (blockPos) {
//                    case RIGHT_POSITION:
//                        robot.setMotorPowers(-0.25, 0.25);
//                        break;
//                    case LEFT_POSITION:
//                        robot.setMotorPowers(0.25, -0.25);
//                        break;
//                    default:
//                        break turn;
//                }
//                sleep(125);
//                robot.setMotorPowers(0, 0);
//            }
//        }
//        telemetry.addData("Func End", "turnToBlock");
//        telemetry.update();
//    }

    //This is for if we turn too far
    private void turnToBlock() {
        telemetry.addData("Func Start", "turnToBlock");
        telemetry.update();
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
                sleep(125);
                robot.setMotorPowers(0, 0);
            }
        }

        endYaw = robot.getYaw();

        telemetry.addData("Func End", "turnToBlock");
        telemetry.update();
    }


//    private void awayFromBlock () {
//        if (blockPos == RIGHT_POSITION || blockPos == LEFT_POSITION)
//            robot.goDistance(-30, 1);
//        else if (blockPos == DEAD_CENTER)
//            robot.goDistance(-60, 1);
//    }

//    private void dropIdol () {
//        robot.moveServo(0);
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        robot.moveServo(1);
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//    }
//
//    private boolean isBetween ( int[] val, int[] min, int[] max) {
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
}
