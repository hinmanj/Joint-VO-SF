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

#include <string.h>
#include <joint_vo_sf.h>


// -------------------------------------------------------------------------------
//								Instructions:
// You need to click on the window of the 3D Scene to be able to interact with it.
// 'n' - Load new frame and solve
// 's' - Turn on/off continuous estimation
// 'e' - Finish/exit
// -------------------------------------------------------------------------------

int main()
{	
    const unsigned int res_factor = 1;
	VO_SF cf(res_factor);


	//Set first image to load, decimation factor and the sequence dir
	unsigned int im_count = 0;
	bool all_precomputed = false;
	const unsigned int decimation = 1; //5
	std::string dir = "D:\\Projects\\Joint-VO-SF\\data\\realsense save\\"; 

	//Load image and create pyramid
	cf.loadImageFromSequence(dir, im_count, res_factor);
	cf.createImagePyramid();

	//Create the 3D Scene
	cf.initializeSceneImageSeq();

	//Auxiliary variables
	int pushed_key = 0;
	bool continuous_exec = false, stop = false;
	
	while (!stop)
	{	
        if (cf.window.keyHit())
            pushed_key = cf.window.getPushedKey();
        else
            pushed_key = 0;

		switch (pushed_key) {
		case 37: //left
			im_count -= decimation;
			cf.updateSceneImageSeq(true, im_count - 1);
			break;
		case 39: //right
			im_count += decimation;
			break;
        //Load new image and solve
        case 'n':
		case 'N':
			im_count += decimation;
			stop = cf.loadImageFromSequence(dir, im_count, res_factor);
            cf.run_VO_SF(true);
            cf.createImagesOfSegmentations();
            cf.updateSceneImageSeq(false, 0);
            break;

		//Start/Stop continuous estimation
		case 's':
		case 'S':
			continuous_exec = !continuous_exec;
			break;
			
		//Close the program
		case 'e':
		case 'E':
			stop = true;
			break;
		}
	
		if ((continuous_exec)&&(!stop))
		{
			im_count += decimation;
			stop = cf.loadImageFromSequence(dir, im_count, res_factor);
			if (stop)
			{
				all_precomputed = true;
				stop = false;
				im_count = 0;
				stop = cf.loadImageFromSequence(dir, im_count, res_factor);
				cf.push_old_frames_back();
				im_count += decimation;
				stop = cf.loadImageFromSequence(dir, im_count, res_factor);
			}
			if (!all_precomputed)
			{
				cf.run_VO_SF(true);
				cf.createImagesOfSegmentations();

				std::string files_dir = "D:\\Projects\\Joint-VO-SF\\data\\realsense save\\";
				char aux[50];
				sprintf(aux, "flow_%d_%d.exr", im_count - 1, im_count);
				std::string flow_name = files_dir + aux;
				cf.saveFlowToExr(flow_name);
			}
			if (all_precomputed)
			{
				cf.push_old_frames_back();
				cf.updateSceneImageSeq(true, im_count - 1);
			}
			else
				cf.updateSceneImageSeq(false, im_count);

		}
	}

	return 0;
}

