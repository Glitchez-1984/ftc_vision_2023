package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Hexagon Detection OpMode", group = "Vision")
public class OpenCvDetection extends OpMode {
    OpenCvCamera webcam;
    Point loc;

    Mat cvtMat = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();

    int cameraHeight = 600;
    int cameraWidth = 800;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(new OpenCvPl());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera open failed");
                telemetry.update();
            }
        });
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }

    class OpenCvPl extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(cvtMat, cvtMat, new Size(3, 3), 0);
            Scalar minValues = new Scalar(0, 0, 200);
            Scalar maxValues = new Scalar(180, 25, 255);
            Core.inRange(cvtMat, minValues, maxValues, mask);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint2f contoursPoly = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly, 3, true);
                Rect boundRect = Imgproc.boundingRect(new MatOfPoint(contoursPoly.toArray()));
                if (boundRect.width > 50 && contoursPoly.toArray().length == 6) {
                    loc = new Point(boundRect.x + boundRect.width / 2, boundRect.y + boundRect.height / 2);
                    telemetry.addData("Hexagon Location", loc);
                    telemetry.addData("Hexagon Size", boundRect.size());
                }
                contoursPoly.release();
            }
            telemetry.update();
            return mask;
        }
    }
}
