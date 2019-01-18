package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

abstract class GenericOpMode extends LinearOpMode {
    GenericOpMode() {}

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
