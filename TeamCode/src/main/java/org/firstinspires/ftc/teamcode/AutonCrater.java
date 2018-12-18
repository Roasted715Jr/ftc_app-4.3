package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater", group = "CompBot")
//@Disabled
public class AutonCrater extends LinearOpMode implements RunningOpMode {

    private AutonProcedures<AutonCrater> autonProcedures = new AutonProcedures<>();
    private Hardware robot = new Hardware();
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(elapsedTime, robot, hardwareMap, AutonProcedures.StartPosition.CRATER, this);
        Thread thread = new Thread(autonProcedures);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        telemetry.clearAll();

        thread.start();
//        autonProcedures.start();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                try {
                    thread.stop();
                } catch (ThreadDeath e) {
                    telemetry.addData("Thread", "Program killed. You murderer");
                    telemetry.update();
                } finally {
                    robot.setMotorPowers(0);
                }
            }

            idle();
        }
    }

    public void displayTelemetry(String msg) {
        telemetry.addData("Auton Crater", msg);
        telemetry.update();
    }
}
