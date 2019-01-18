package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.Gen;

@Autonomous(name = "Depot", group = "CompBot")
public class AutonDepot extends GenericOpMode {

    private AutonProcedures<AutonDepot> autonProcedures = new AutonProcedures<>();
    private Hardware<AutonDepot> robot = new Hardware<>(this);
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(elapsedTime, robot, hardwareMap, AutonProcedures.DEPOT_START, this);

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
                thread.stop();
                robot.stop();
            }

            idle();
        }
    }

    public void displayTelemetry(String msg) {
        telemetry.addData("Auton Depot", msg);
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
