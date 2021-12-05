package org.firstinspires.ftc.teamcode.subsystem.visionsystem;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*The image processing used by OpenCV to process the images from the webcam*/
class BarCodeDetectPipeline extends OpenCvPipeline
{
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    Mat mask = new Mat();
    Mat input_hsv = new Mat();
    Mat input_bgr = new Mat();
    Mat cropped = new Mat();
    Scalar sumValue = new Scalar(0);
    float totalPixs = 0.0f;
    float[] sumValNorm = new float[3];

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */
//            USE LOWERlim ~60-70, UPPERLIM ~80-90 for green detection
        Imgproc.cvtColor(input, input_bgr,Imgproc.COLOR_RGBA2BGR); //EasyOpenCV return images in RGBA format
        Imgproc.cvtColor(input_bgr, input_hsv, Imgproc.COLOR_BGR2HSV); // We convert them to BGR since only BGR (or RGB) conversions to HSV exist
        Core.inRange(input_hsv,
                new Scalar(60,50,50),
                new Scalar(80,255,255),
                mask);
        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        for (int i=0; i<3; i++) {
            Imgproc.rectangle(
                    input,
                    new Point(
                            //0,0),
                            (i+1) * input.cols() * (1f / 5f),
                            //input.cols()/4,
                            input.rows() / 4),
                    //input.rows()/4),
                    new Point(
                            (i + 2) * input.cols() * (1f / 5f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 0, 255), 4);

            cropped = mask.submat(
                    new org.opencv.core.Range((int) (input.rows() / 4), (int) (input.rows() * (3f / 4f))),
                    new org.opencv.core.Range((int)((i+1) * input.cols() * (1f / 5f)), (int) ((i+2)*input.cols() * (1f / 5f)))
            );

            //Sum all the color thresholded pixels to detect the color of interest
            sumValue = Core.sumElems(cropped);
            totalPixs = (float) (cropped.cols() * cropped.rows());
            sumValNorm[i] = (float) (sumValue.val[0]) / totalPixs / 255.0f; //I might have the scaling wrong
        }

        return input; //this is what will be displayed in the DS preview pane.
//            return mask; //Display the thresholded mask to check color threshold limits
    }

    public int FindObjectPosition()
    {
        //Assume the object is on position 0, but change if more pixels found elsewhere
        int ind = 0;
        for(int i = 1; i < 3; i++)
        {
            if(sumValNorm[i] > sumValNorm[ind])
            {
                ind = i;
            }
        }
        return ind;
    }
}