package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Depot", group = "CompBot")
//@Disabled
public class AutonDepot extends LinearOpMode implements RunningOpMode {

    private AutonProcedures<AutonDepot> autonProcedures = new AutonProcedures<>();
    private Hardware robot = new Hardware();
    private ElapsedTime elapsedTime = new ElapsedTime();
    private Runnable auton;

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(elapsedTime, robot, hardwareMap, AutonProcedures.StartPosition.DEPOT, this);
        auton = new Runnable() {
            @Override
            public void run() {
                autonProcedures.start();
            }
        };

        Thread thread = new Thread(auton);

        waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        telemetry.clearAll();

        thread.start();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                robot.stop();
                robot.setMotorPowers(0);
            }

            idle();
        }
    }

    public void displayTelemetry(String msg) {
        telemetry.addData("Auton Depot", msg);
        telemetry.update();
    }
}
