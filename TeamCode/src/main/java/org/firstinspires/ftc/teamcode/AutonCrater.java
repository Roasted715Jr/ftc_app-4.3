package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater", group = "CompBot")
public class AutonCrater extends LinearOpMode implements RunningOpMode {

    private AutonProcedures<AutonCrater> autonProcedures = new AutonProcedures<>();
    private Hardware robot = new Hardware();
    private ElapsedTime elapsedTime = new ElapsedTime();
    private Runnable auton;

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(elapsedTime, robot, hardwareMap, AutonProcedures.StartPosition.CRATER, this);

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autonProcedures.start();
            }
        });

        waitForStart();

        telemetry.clearAll();

        thread.start();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                robot.stop();
                thread.stop();
            }

            idle();
        }
    }

    public void displayTelemetry(String msg) {
        telemetry.addData("Auton Crater", msg);
        telemetry.update();
    }

    @Override
    public void waitForStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
    }
}
