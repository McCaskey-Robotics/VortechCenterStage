package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-64, -36, 0);
        drive.setPoseEstimate(startPose);

        Trajectory zone1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40,-36), Math.PI/2)
                .build();

        Trajectory zone2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40,-36), 0)
                .build();

        Trajectory zone3 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40,-36), -Math.PI/2)
                .build();

        Trajectory zoneUnknown = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0,0), -Math.PI/2)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        OpenCvPipeline pipeline = new SignalPipeline();
        boolean openCvInitFail[] = {false};

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                cam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                cam.setPipeline(pipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                openCvInitFail[0] = true;
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        cam.showFpsMeterOnViewport(false);

        while(!isStarted()) {
            if(openCvInitFail[0]) {
                telemetry.addLine("WARNING: OpenCV did not open correctly");
            }

            int zone = SignalPipeline.getAnalysis();

            if (zone == 1) {
                telemetry.addLine("Zone 1 (LEFT)");
            }
            else if (zone == 2) {
                telemetry.addLine("Zone 2 (MID)");
            }
            else if (zone == 3) {
                telemetry.addLine("Zone 3 (RIGHT)");
            }
            else {
                telemetry.addLine("Zone Unknown");
            }
            telemetry.update();
        }

        if(opModeIsActive()) {
            int zone = SignalPipeline.getAnalysis();

            Trajectory backBoard = drive.trajectoryBuilder(zone2.end())
                    .lineTo(new Vector2d(-30 - 6*zone, -36))
                    .lineTo(new Vector2d(-30 - 6*zone, 50))
                    .build();

            Trajectory park = drive.trajectoryBuilder(backBoard.end())
                    .splineTo(new Vector2d(54, -12), Math.PI/2)
                    .build();

            //Drive to the correct zone
            if (zone == 1) {
                drive.followTrajectoryAsync(zone1);
            }
            else if (zone == 2) {
                drive.followTrajectoryAsync(zone2);
            }
            else if (zone == 3) {
                drive.followTrajectoryAsync(zone3);
            }
            else {
                drive.followTrajectoryAsync(zoneUnknown);
            }

            //Drive to the backboard and the correct pix pos
            drive.followTrajectoryAsync(backBoard);

            //Park
            drive.followTrajectoryAsync(park);

        }

    }
}
