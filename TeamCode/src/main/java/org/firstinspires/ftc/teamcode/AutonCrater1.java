package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "Crater", group = "CompBot")
@Disabled
public class AutonCrater1 extends LinearOpMode {
    private static final int BLOCK_NOT_FOUND = 0;
    private static final int RIGHT_POSITION = 1;
    private static final int CENTER_POSITION = 2;
    private static final int LEFT_POSITION = 3;
    private static final int DEAD_CENTER = 4;

    private Hardware robot = new Hardware();

    private int cCounter, dcCounter, lCounter, rCounter, blockPos = BLOCK_NOT_FOUND;

    private GoldVision goldVision;
    //    private CloseableVuforiaLocalizer vuforia;
    private VuforiaLocalizer vuforia;

    private ElapsedTime elapsedTime = new ElapsedTime();

    //Distance to center block is 7sqrt(2)/4 ft
    //Distance to corner blocks is sqrt(130)/2

    @Override
    public void runOpMode() {
        goldVision = new GoldVision();
        robot.init(hardwareMap);

        rCounter = 0;
        dcCounter = 0;
        cCounter = 0;
        lCounter = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Abq1tHr/////AAABmYC8ioniS0f2gyQRx7fZlTWMwyYcrV/bnslJvcDe0AhxA/GAkYTIdNbPWjYtplipzvASUZRGR+AoGDI1dKyuCFCc4qy1eVbx8NO4nuAKzeGoncY7acvfol19suW5Zl29E+APEV0CG4GVBe4R+bZ/Xyd2E7CZ7AcrLbWM8+SJiMCDnxJa3J0ozBHMPMs6GNFyYS6YCVNMkFcLEKxDicwXqpuJddG5XenbAs8ot9UT11WRYZjpprLkSRtM1/OyigcUeb0wk2PL6lFVBMHMZbWK5HkJEmBoN5+v2fP6zouj0GPGyEh/eV8Xe71LhBz0WXKd180hUCowZVBfdsTtuYwFiBkAyRLtiQQb4/b80sAx1b6s";
        robot.initCamera();
        parameters.cameraName = robot.getWebcam();

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        vuforia = (CloseableVuforiaLocalizer) ClassFactory.getInstance().createVuforia(parameters);
//        vuforia = new CloseableVuforiaLocalizer(parameters);
        vuforia.enableConvertFrameToBitmap();

        waitForStart();
        telemetry.clearAll();

//        blockPos = getBlockPos(1000);
//        telemetry.addData("blockPos", blockPos);
//        telemetry.update();

        goToBlock();

            while (opModeIsActive()) {
                idle();
            }
    }

    void goToBlock (){
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

    private int getBlockPos ( int mSec){
        int blockPos = 0;

        lCounter = 0;
        cCounter = 0;
        dcCounter = 0;
        rCounter = 0;

        elapsedTime.reset();

        while (elapsedTime.milliseconds() <= mSec) {
            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
                @Override
                public void accept(Frame frame) {
                    Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                    Mat img = new Mat();
                    if (bitmap != null) {
                        Utils.bitmapToMat(bitmap, img);
                    }

                    img = goldVision.processFrame(img, null);

                    List<MatOfPoint> contours = goldVision.getContours();

                    for (int i = 0; i < contours.size(); i++) {
                        // get the bounding rectangle of a single contour, we use it to get the x/y center
                        // yes there's a mass center using Imgproc.moments but w/e
                        Rect boundingRect = Imgproc.boundingRect(contours.get(i));

                        double boundingRectX = (boundingRect.x + boundingRect.width) / 2;
                        double boundingRectY = (boundingRect.y + boundingRect.height) / 2;

//                        if (boundingRectY <= 115) {
                        if (boundingRectX < 100)
                            rCounter++; //These are reversed now that the camera will be upside down
                        else if (boundingRectX > 185 && boundingRectX < 215)
                            dcCounter++;
                        else if (boundingRectX > 120 && boundingRectX < 280)
                            cCounter++;
                        else if (boundingRectX > 300)
                            lCounter++;
//                        }
                    }
                }
            }));
        }

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

        return blockPos;
    }

    private void turnToBlock () {
        if (blockPos != BLOCK_NOT_FOUND) {
            turn:
            while (getBlockPos(100) != DEAD_CENTER) {
                //Move motors then stop them after a short amount of time
                switch (blockPos) {
                    case RIGHT_POSITION:
                        robot.setMotorPowers(-0.125, 0.125);
                        break;
                    case LEFT_POSITION:
                        robot.setMotorPowers(0.125, -0.125);
                        break;
                    default:
                        break turn;
                }
                try {
                    Thread.sleep(25);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.setMotorPowers(0, 0);
            }
        }
    }

    private void awayFromBlock () {
        if (blockPos == RIGHT_POSITION || blockPos == LEFT_POSITION)
            robot.goDistance(-30, 1);
        else if (blockPos == DEAD_CENTER)
            robot.goDistance(-60, 1);
    }
}
