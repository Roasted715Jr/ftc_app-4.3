package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater", group = "CompBot")
//@Disabled
public class AutonCrater extends LinearOpMode {

    private AutonProcedures autonProcedures = new AutonProcedures();
    private Hardware robot = new Hardware();
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(elapsedTime, robot, hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        telemetry.clearAll();

        autonProcedures.start(AutonProcedures.StartPosition.CRATER);

        while (opModeIsActive()) {
            idle();
        }
    }
}
