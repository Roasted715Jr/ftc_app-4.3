package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "Depot", group = "CompBot")
@Disabled
public abstract class Auton extends LinearOpMode {
//    private Hardware robot = new Hardware();
//    private AutonProcedures autonProcedures = new AutonProcedures();
//    private VuforiaLocalizer vuforia;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        autonProcedures.init(robot);
//
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = "Abq1tHr/////AAABmYC8ioniS0f2gyQRx7fZlTWMwyYcrV/bnslJvcDe0AhxA/GAkYTIdNbPWjYtplipzvASUZRGR+AoGDI1dKyuCFCc4qy1eVbx8NO4nuAKzeGoncY7acvfol19suW5Zl29E+APEV0CG4GVBe4R+bZ/Xyd2E7CZ7AcrLbWM8+SJiMCDnxJa3J0ozBHMPMs6GNFyYS6YCVNMkFcLEKxDicwXqpuJddG5XenbAs8ot9UT11WRYZjpprLkSRtM1/OyigcUeb0wk2PL6lFVBMHMZbWK5HkJEmBoN5+v2fP6zouj0GPGyEh/eV8Xe71LhBz0WXKd180hUCowZVBfdsTtuYwFiBkAyRLtiQQb4/b80sAx1b6s";
//        robot.initCamera();
//        parameters.cameraName = robot.getWebcam();
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        vuforia.enableConvertFrameToBitmap();
//
//        waitForStart();
//
//        telemetry.addData("Running", "Turn to block");
//        telemetry.update();
//        autonProcedures.goToBlock(hardwareMap);
//        telemetry.addData("Completed", "Turned to block");
//
//        //Turn only when the block is not in the center using the gyro
//
//        if (autonProcedures.blockPos == AutonProcedures.RIGHT_POSITION) {
//            //Right side
//            telemetry.addData("Running", "Turn to depot");
//            telemetry.update();
//            robot.goDistance(2.25, 1.125, 1, 0.5); //Was 10.5
//            //Go some other distance
//            telemetry.addData("Completed", "Turned to depot");
//        } else if (autonProcedures.blockPos == AutonProcedures.LEFT_POSITION) {
//            telemetry.addData("Running", "Turn to depot");
//            telemetry.update();
//            robot.goDistance(1.125, 2.25, 0.5, 1);
//            telemetry.addData("Completed", "Turned to depot");
//        }
//
//        telemetry.addData("Running", "Drive to depot");
//        telemetry.update();
//        autonProcedures.driveIntoDepot(0.6);
//        telemetry.addData("Completed", "Drive to depot");
//
//        telemetry.addData("Running", "Drop idol");
//        telemetry.update();
//        robot.goDistance(5, 0.6); //Was 17 inches
//        autonProcedures.dropIdol();
//        telemetry.addData("Completed", "Dropped idol");
//
//        telemetry.addData("Running", "Reverse from idol");
//        telemetry.update();
//        robot.goDistance(-6, 1); //Was -4
//        telemetry.addData("Completed", "Reversed from idol");
//        telemetry.update();
//    }
}
