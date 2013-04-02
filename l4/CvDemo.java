/**
 * <p>Demo of CvBase.</p>
 *
 * <p>This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.</p>
 *
 * <p>This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.</p>
 *
 * <p>You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA</p>
 *
 * <p>Copyright (c) 2011 Marsette A. Vona</p>
 **/

package l4;

import com.googlecode.javacpp.*;
import com.googlecode.javacv.*;
import com.googlecode.javacv.cpp.*;
import static com.googlecode.javacv.cpp.opencv_calib3d.*;
import static com.googlecode.javacv.cpp.opencv_contrib.*;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_features2d.*;
import static com.googlecode.javacv.cpp.opencv_flann.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_legacy.*;
import static com.googlecode.javacv.cpp.opencv_ml.*;
import static com.googlecode.javacv.cpp.opencv_objdetect.*;
import static com.googlecode.javacv.cpp.opencv_video.*;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

/**
 * <p>Demo of {@link CvBase}.</p>
 *
 * @author Marsette A. Vona
 **/
public class CvDemo extends CvBase {
  
  private static final String svnid =
    "$Id: CvDemo.java 1868 2013-03-14 17:56:36Z vona $";

  /** sets options for the demo **/
  public CvDemo() {
    super("CvDemo");
    useWindow = false;
    useCanvasFrame = !java.awt.GraphicsEnvironment.isHeadless();
    useConsole = true;
    useServer = true;
    brightness = 0.3;
    maxFPS = 30;
  }

  /** shows how to do special config **/
  public int initExt(int argc, String argv[], int ate) {
//    v4l2DisableAuto();
    return ate;
  }
  
  /** Program entry point. **/
  public static void main(String argv[]) {
    CvDemo d = new CvDemo();
    System.out.println("ate "+d.init(argv.length, argv)+" args");
    d.mainLoop();
    d.release();
  }
}
