package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by ftc on 2/20/2017.
 */

public class VuforiaUtil {
    public static VectorF anglesFromTarget(OpenGLMatrix pose ) {
        if(pose != null) {
            float[] data = pose.getData();
            float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
            double thetaX = 360 * Math.atan2(rotation[2][1], rotation[2][2]) / (2 * Math.PI);
            double thetaY = 360 * Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2])) / (2 * Math.PI);
            double thetaZ = 360 * Math.atan2(rotation[1][0], rotation[0][0]) / (2 * Math.PI);
            return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
        }
        else {
            return null;
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    public static String formatOpenGLMatrix(OpenGLMatrix transformationMatrix){
        return transformationMatrix.formatAsTransform();
    }
}
