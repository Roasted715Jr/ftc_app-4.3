package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater", group = "Rover Ruckus")
public class AutonCrater extends GenericOpMode {

    private AutonProcedures<AutonCrater> autonProcedures = new AutonProcedures<>();
    private Hardware<AutonCrater> robot = new Hardware<>(this);
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(elapsedTime, robot, hardwareMap, AutonProcedures.CRATER_START, this);

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
}
