package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the Power Play Beacon when lined up with
 * the sample regions over the first 3 stones.
 */
public class SignalPipeline extends OpenCvPipeline {
    //Set coords of bounding boxes
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50, 290);
    static final Point Region2_TOPLEFT_ANCHOR_POINT = new Point(610, 280);
    static final Point Region3_TOPLEFT_ANCHOR_POINT = new Point(1130, 300);
    static final int REGION_WIDTH = 100;
    static final int REGION_HEIGHT = 200;

    //Set points for submats
    Point pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point pointC = new Point(
            Region2_TOPLEFT_ANCHOR_POINT.x,
            Region2_TOPLEFT_ANCHOR_POINT.y + 20);
    Point pointD = new Point(
            Region2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH - 10,
            Region2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT - 20);
    Point pointE = new Point(
            Region3_TOPLEFT_ANCHOR_POINT.x,
            Region3_TOPLEFT_ANCHOR_POINT.y);
    Point pointF = new Point(
            Region3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            Region3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Color definitions
    private final Scalar
            BLUE = new Scalar(0, 0, 255),
            RED = new Scalar(255, 0, 0);

    Mat region1_Cr;
    Mat region2_Cr;
    Mat region3_Cr;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    int avgCr;
    static int signal;

    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCr(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cr = Cr.submat(new Rect(pointA, pointB));
        region2_Cr = Cr.submat(new Rect(pointC, pointD));
        region3_Cr = Cr.submat(new Rect(pointE, pointF));
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat areaMat1 = input.submat(new Rect(pointA, pointB));
        Scalar meanColors1 = Core.mean(areaMat1);
        Mat areaMat2 = input.submat(new Rect(pointC, pointD));
        Scalar meanColors2 = Core.mean(areaMat2);
        Mat areaMat3 = input.submat(new Rect(pointE, pointF));
        Scalar meanColors3 = Core.mean(areaMat3);

        Imgproc.putText(
                input,
                "Colors: " + Math.round(meanColors1.val[0]) + ", " + Math.round(meanColors1.val[1]) + ", " + Math.round(meanColors1.val[2]),
                new Point(100, 600),
                0,
                1,
                new Scalar(0, 0, 0),
                2);

        Imgproc.putText(
                input,
                "Colors: " + Math.round(meanColors2.val[0]) + ", " + Math.round(meanColors2.val[1]) + ", " + Math.round(meanColors2.val[2]),
                new Point(100, 630),
                0,
                1,
                new Scalar(0, 0, 0),
                2);

        Imgproc.rectangle(
                input,
                pointA,
                pointB,
                new Scalar (255, 0, 0),
                2
        );
        Imgproc.rectangle(
                input,
                pointC,
                pointD,
                new Scalar (255, 0, 0),
                2
        );
        Imgproc.rectangle(
                input,
                pointE,
                pointF,
                new Scalar (255, 0, 0),
                2
        );


        // Change the bounding box color based on the sleeve color
        if (meanColors1.val[2] > (meanColors1.val[0] + meanColors1.val[1])/1.2 || meanColors1.val[0] > (meanColors1.val[1] + meanColors1.val[2])/1.2) {
            signal = 1;
            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    new Scalar (0, 255, 0),
                    2
            );
        } else if (meanColors2.val[2] > (meanColors2.val[0] + meanColors2.val[1])/1.2 || meanColors2.val[0] > (meanColors2.val[1] + meanColors2.val[2])/1.2) {
            signal = 2;
            Imgproc.rectangle(
                    input,
                    pointC,
                    pointD,
                    new Scalar (0, 255, 0),
                    2
            );
        } else if (meanColors3.val[2] > (meanColors3.val[0] + meanColors3.val[1])/1.2 || meanColors3.val[0] > (meanColors3.val[1] + meanColors3.val[2])/1.2){
            signal = 3;
            Imgproc.rectangle(
                    input,
                    pointE,
                    pointF,
                    new Scalar (0, 255, 0),
                    2
            );
        } else {
            signal = 0;
        }


        // Release and return input
        areaMat1.release();
        areaMat2.release();
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public static int getAnalysis() {
        return signal;
    }
}

