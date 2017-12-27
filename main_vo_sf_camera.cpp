/*********************************************************************************
**Fast Odometry and Scene Flow from RGB-D Cameras based on Geometric Clustering	**
**------------------------------------------------------------------------------**
**																				**
**	Copyright(c) 2017, Mariano Jaimez Tarifa, University of Malaga & TU Munich	**
**	Copyright(c) 2017, Christian Kerl, TU Munich								**
**	Copyright(c) 2017, MAPIR group, University of Malaga						**
**	Copyright(c) 2017, Computer Vision group, TU Munich							**
**																				**
**  This program is free software: you can redistribute it and/or modify		**
**  it under the terms of the GNU General Public License (version 3) as			**
**	published by the Free Software Foundation.									**
**																				**
**  This program is distributed in the hope that it will be useful, but			**
**	WITHOUT ANY WARRANTY; without even the implied warranty of					**
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the				**
**  GNU General Public License for more details.								**
**																				**
**  You should have received a copy of the GNU General Public License			**
**  along with this program. If not, see <http://www.gnu.org/licenses/>.		**
**																				**
*********************************************************************************/

#include <joint_vo_sf.h>
#include <camera.h>
#include <opencv2/opencv.hpp>
#include <string>


// -------------------------------------------------------------------------------
//								Instructions:
// You need to click on the window of the 3D Scene to be able to interact with it.
// 'n' - Capture new frame and show it (but don't run the algorithm)
// 'a' - Run the algorithm with the last two frames captured
// 's' - Turn on/off continuous estimation
// 'r' - Reset the camera pose
// 'e' - Finish/exit
// -------------------------------------------------------------------------------

int main()
{	
    unsigned int res_factor = 1;
	VO_SF cf(res_factor);
	RealSense_Camera camera(res_factor);

	//Create the 3D Scene
	cf.initializeSceneCamera();

	//Initialize camera and method
    camera.openCamera();
    camera.disableAutoExposureAndWhiteBalance();
	camera.loadFrame(cf.depth_wf, cf.intensity_wf);
    cf.createImagePyramid();
	camera.loadFrame(cf.depth_wf, cf.intensity_wf);
    cf.createImagePyramid();
	cf.initializeKMeans();

	//Auxiliary variables for the interface
	int pushed_key = 0;
	bool anything_new = false, stop = false;
    bool clean_sf = false, continuous_exec = false;
	bool recording = false;
	int save_file_index = 0;

	
	while (!stop)
	{	

        if (cf.window.keyHit())
            pushed_key = cf.window.getPushedKey();
        else
            pushed_key = 0;

		switch (pushed_key) {
			
		case 32: //space
			recording = !recording;
			continuous_exec = !recording;
			break;


        //Capture a new frame
		case  'n':
		case 'N':
			cf.use_b_temp_reg = false; //I turn it off here for individual framepair tests
            camera.loadFrame(cf.depth_wf, cf.intensity_wf);
			cf.createImagePyramid();
			cf.kMeans3DCoord();
            cf.createImagesOfSegmentations();

            anything_new = true;
            clean_sf = true;
			break;

        //Compute the solution
        case 'a':
		case 'A':
            cf.run_VO_SF(false);
            cf.createImagesOfSegmentations();

            anything_new = true;
            break;

        //Turn on/off continuous estimation
        case 's':
		case 'S':
            continuous_exec = !continuous_exec;
            break;

		//Reset the camera pose
		case 'r':
		case 'R':
			cf.cam_pose.setFromValues(0,0,1.5,0,0,0);
			anything_new = true;
			break;
			
		//Close the program
		case 'e':
		case 'E':
			stop = true;
			break;
		}

		if (recording)
		{
			std::string files_dir = "D:\\Projects\\Joint-VO-SF\\data\\realsense save\\";
			char daux[50];
			char caux[50];
			sprintf(daux, "d%d.png", save_file_index);
			sprintf(caux, "c%d.png", save_file_index);
			std::string depth_name = files_dir + daux;
			std::string color_name = files_dir + caux;

			camera.save_new_frames(depth_name, color_name);
			save_file_index++;
		}
        if (continuous_exec)
        {
            camera.loadFrame(cf.depth_wf, cf.intensity_wf);
            cf.run_VO_SF(true);
            cf.createImagesOfSegmentations();
            anything_new = 1;
        }
	
		if (anything_new)
		{
			cf.updateSceneCamera(clean_sf);
			clean_sf = false;
			anything_new = 0;
		}
	}

    camera.closeCamera();
	return 0;
}

