#include"HandModel.h"
void HandModel::load_faces(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	f >> NumofFaces;
	FaceIndex = Eigen::MatrixXf::Zero(NumofFaces, 3);
	for (int i = 0; i < NumofFaces; ++i) {
		f >> FaceIndex(i, 0) >> FaceIndex(i, 1) >> FaceIndex(i, 2);
	}
	f.close();
	printf("Load Faces succeed!!!\n");
	std::cout << "num of Face is: " << NumofFaces << std::endl;
}

void HandModel::load_vertices(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	f >> NumofVertices;
	Vectices = Eigen::MatrixXf::Zero(NumofVertices, 3);
	Vertices_normal = Eigen::MatrixXf::Zero(NumofVertices, 3);
	for (int i = 0; i < NumofVertices; ++i) {
		f >> Vectices(i, 0) >> Vectices(i, 1) >> Vectices(i, 2);
	}
	f.close();
	printf("Load vertices succeed!!!\n");
	std::cout << "num of vertices is: " << NumofVertices << std::endl;
}

void HandModel::load_weight(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	Weights = Eigen::MatrixXf::Zero(NumofVertices, NumofJoints);
	for (int i = 0; i < NumofVertices; ++i) {
		for (int j = 0; j < NumofJoints; ++j)
		{
			f >> Weights(i, j);
		}
	}
	f.close();
	printf("Load weights succeed!!!\n");
}

HandModel::HandModel()
{
	NumofJoints = 21;
	Joints = new Joint[21];
	//wrist
	 {
		Joints[0].joint_name = "Wrist";
		Joints[0].joint_index = 0;
		Joints[0].GlobalInitPosition << 0.0f, 0.0f, 0.0f, 1.0f;
		Joints[0].ChildGlobalInitPosition << -0.466003f, 81.5657f, -2.2004f, 1.0f;
		Joints[0].parent_joint_index = -1;

		Joints[0].params_length = 6;
		Joints[0].params_index = new int[6]; Joints[0].params_type = new int[6];
		Joints[0].params_index[0] = 0; Joints[0].params_type[0] = dof_type(x_axis_trans);
		Joints[0].params_index[1] = 1; Joints[0].params_type[1] = dof_type(y_axis_trans);
		Joints[0].params_index[2] = 2; Joints[0].params_type[2] = dof_type(z_axis_trans);
		Joints[0].params_index[3] = 3; Joints[0].params_type[3] = dof_type(x_axis_rotate);
		Joints[0].params_index[4] = 4; Joints[0].params_type[4] = dof_type(y_axis_rotate);
		Joints[0].params_index[5] = 5; Joints[0].params_type[5] = dof_type(z_axis_rotate);
	}

	//thumb
	{
		{
			Joints[1].joint_name = "ThumbLower";
			Joints[1].joint_index = 1;
			Joints[1].GlobalInitPosition << -28.468f, 27.4715f, -18.4322f, 1.0f;
			Joints[1].ChildGlobalInitPosition << -57.3719f, 44.0606f, -31.1096f, 1.0f;
			Joints[1].parent_joint_index = 0;

			Joints[1].params_length = 2;
			Joints[1].params_index = new int[2]; Joints[1].params_type = new int[2];
			Joints[1].params_index[0] = 6; Joints[1].params_type[0] = dof_type(y_axis_rotate);
			Joints[1].params_index[1] = 7; Joints[1].params_type[1] = dof_type(z_axis_rotate);
		}

		{
			Joints[2].joint_name = "ThumbMiddle";
			Joints[2].joint_index = 2;
			Joints[2].GlobalInitPosition << -57.3719f, 44.0606f, -31.1096f, 1.0f;
			Joints[2].ChildGlobalInitPosition << -74.4329f, 66.375f, -44.2624f, 1.0f;
			Joints[2].parent_joint_index = 1;

			Joints[2].params_length = 1;
			Joints[2].params_index = new int[1]; Joints[2].params_type = new int[1];
			Joints[2].params_index[0] = 8; Joints[2].params_type[0] = dof_type(z_axis_rotate);
		}

		{
			Joints[3].joint_name = "ThumbTop";
			Joints[3].joint_index = 3;
			Joints[3].GlobalInitPosition << -74.4329f, 66.375f, -44.2624f, 1.0f;
			Joints[3].ChildGlobalInitPosition << -85.4636f, 88.6297f, -58.3764f, 1.0f;
			Joints[3].parent_joint_index = 2;

			Joints[3].params_length = 1;
			Joints[3].params_index = new int[1]; Joints[3].params_type = new int[1];
			Joints[3].params_index[0] = 9; Joints[3].params_type[0] = dof_type(z_axis_rotate);
		}

		{
			Joints[4].joint_name = "ThumbSite";
			Joints[4].joint_index = 4;
			Joints[4].GlobalInitPosition << -85.4636f, 88.6297f, -58.3764f, 1.0f;
			Joints[4].HasChild = false;
			Joints[4].parent_joint_index = 3;

			Joints[4].params_length = 0;
			Joints[4].params_index = NULL; Joints[4].params_type = NULL;
		}
	}

	//index
	{
		{
			Joints[5].joint_name = "IndexLower";
			Joints[5].joint_index = 5;
			Joints[5].GlobalInitPosition << -25.29f, 82.3287f, -2.863f, 1.0f;
			Joints[5].ChildGlobalInitPosition << -30.185f, 120.669f, -5.29089f, 1.0f;
			Joints[5].parent_joint_index = 0;

			Joints[5].params_length = 2;
			Joints[5].params_index = new int[2]; Joints[5].params_type = new int[2];
			Joints[5].params_index[0] = 10; Joints[5].params_type[0] = dof_type(y_axis_rotate);
			Joints[5].params_index[1] = 11; Joints[5].params_type[1] = dof_type(z_axis_rotate);
		}

		{
			Joints[6].joint_name = "IndexMiddle";
			Joints[6].joint_index = 6;
			Joints[6].GlobalInitPosition << -30.185f, 120.669f, -5.29089f, 1.0f;
			Joints[6].ChildGlobalInitPosition << -32.2498f, 143.564f, -14.5545f, 1.0f;
			Joints[6].parent_joint_index = 5;

			Joints[6].params_length = 1;
			Joints[6].params_index = new int[1]; Joints[6].params_type = new int[1];
			Joints[6].params_index[0] = 12; Joints[6].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[7].joint_name = "IndexTop";
			Joints[7].joint_index = 7;
			Joints[7].GlobalInitPosition << -32.2498f, 143.564f, -14.5545f, 1.0f;
			Joints[7].ChildGlobalInitPosition << -33.5694f, 165.174f, -24.3519f, 1.0f;
			Joints[7].parent_joint_index = 6;

			Joints[7].params_length = 1;
			Joints[7].params_index = new int[1]; Joints[7].params_type = new int[1];
			Joints[7].params_index[0] = 13; Joints[7].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[8].joint_name = "IndexSite";
			Joints[8].joint_index = 8;
			Joints[8].GlobalInitPosition << -33.5694f, 165.174f, -24.3519f, 1.0f;
			Joints[8].HasChild = false;
			Joints[8].parent_joint_index = 7;

			Joints[8].params_length = 0;
			Joints[8].params_index = NULL; Joints[8].params_type = NULL;
		}
	}

	//middle
	{
		{
			Joints[9].joint_name = "MiddleLower";
			Joints[9].joint_index = 9;
			Joints[9].GlobalInitPosition << -0.466003f, 81.5657f, -2.2004f, 1.0f;
			Joints[9].ChildGlobalInitPosition << 0.555016f, 127.541f, -6.19035f, 1.0f;
			Joints[9].parent_joint_index = 0;

			Joints[9].params_length = 2;
			Joints[9].params_index = new int[2]; Joints[9].params_type = new int[2];
			Joints[9].params_index[0] = 14; Joints[9].params_type[0] = dof_type(y_axis_rotate);
			Joints[9].params_index[1] = 15; Joints[9].params_type[1] = dof_type(z_axis_rotate);
		}

		{
			Joints[10].joint_name = "MiddleMiddle";
			Joints[10].joint_index = 10;
			Joints[10].GlobalInitPosition << 0.555016f, 127.541f, -6.19035f, 1.0f;
			Joints[10].ChildGlobalInitPosition << -1.544f, 152.677f, -18.7633f, 1.0f;
			Joints[10].parent_joint_index = 9;

			Joints[10].params_length = 1;
			Joints[10].params_index = new int[1]; Joints[10].params_type = new int[1];
			Joints[10].params_index[0] = 16; Joints[10].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[11].joint_name = "MiddleTop";
			Joints[11].joint_index = 11;
			Joints[11].GlobalInitPosition << -1.544f, 152.677f, -18.7633f, 1.0f;
			Joints[11].ChildGlobalInitPosition << -3.97699f, 175.197f, -32.1354f, 1.0f;
			Joints[11].parent_joint_index = 10;

			Joints[11].params_length = 1;
			Joints[11].params_index = new int[1]; Joints[11].params_type = new int[1];
			Joints[11].params_index[0] = 17; Joints[11].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[12].joint_name = "MiddleSite";
			Joints[12].joint_index = 12;
			Joints[12].GlobalInitPosition << -3.97699f, 175.197f, -32.1354f, 1.0f;
			Joints[12].HasChild = false;
			Joints[12].parent_joint_index = 11;

			Joints[12].params_length = 0;
			Joints[13].params_index = NULL; Joints[13].params_type = NULL;
		}
	}

	//Ring
	{
		{
			Joints[13].joint_name = "RingLower";
			Joints[13].joint_index = 13;
			Joints[13].GlobalInitPosition << 18.433f, 78.3977f, -5.57517f, 1.0f;
			Joints[13].ChildGlobalInitPosition << 25.829f, 121.182f, -10.9752f, 1.0f;
			Joints[13].parent_joint_index = 0;

			Joints[13].params_length = 2;
			Joints[13].params_index = new int[2]; Joints[13].params_type = new int[2];
			Joints[13].params_index[0] = 18; Joints[13].params_type[0] = dof_type(y_axis_rotate);
			Joints[13].params_index[1] = 19; Joints[13].params_type[1] = dof_type(z_axis_rotate);
		}

		{
			Joints[14].joint_name = "RingMiddle";
			Joints[14].joint_index = 14;
			Joints[14].GlobalInitPosition << 25.829f, 121.182f, -10.9752f, 1.0f;
			Joints[14].ChildGlobalInitPosition << 25.771f, 141.244f, -25.8824f, 1.0f;
			Joints[14].parent_joint_index = 13;

			Joints[14].params_length = 1;
			Joints[14].params_index = new int[1]; Joints[14].params_type = new int[1];
			Joints[14].params_index[0] = 20; Joints[14].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[15].joint_name = "RingTop";
			Joints[15].joint_index = 15;
			Joints[15].GlobalInitPosition << 25.771f, 141.244f, -25.8824f, 1.0f;
			Joints[15].ChildGlobalInitPosition << 25.114f, 159.881f, -41.3909f, 1.0f;
			Joints[15].parent_joint_index = 14;

			Joints[15].params_length = 1;
			Joints[15].params_index = new int[1]; Joints[15].params_type = new int[1];
			Joints[15].params_index[0] = 21; Joints[15].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[16].joint_name = "RingSite";
			Joints[16].joint_index = 16;
			Joints[16].GlobalInitPosition << 25.114f, 159.881f, -41.3909f, 1.0f;
			Joints[16].HasChild = false;
			Joints[16].parent_joint_index = 15;

			Joints[16].params_length = 0;
			Joints[16].params_index = NULL; Joints[16].params_type = NULL;
		}
	}

	//Pinkey
	{

		{
			Joints[17].joint_name = "PinkeyLower";
			Joints[17].joint_index = 17;
			Joints[17].GlobalInitPosition << 35.353f, 70.1487f, -9.93717f, 1.0f;
			Joints[17].ChildGlobalInitPosition << 45.5f, 96.8537f, -17.9839f, 1.0f;
			Joints[17].parent_joint_index = 0;

			Joints[17].params_length = 2;
			Joints[17].params_index = new int[2]; Joints[17].params_type = new int[2];
			Joints[17].params_index[0] = 22; Joints[17].params_type[0] = dof_type(y_axis_rotate);
			Joints[17].params_index[1] = 23; Joints[17].params_type[1] = dof_type(z_axis_rotate);
		}

		{
			Joints[18].joint_name = "PinkeyMiddle";
			Joints[18].joint_index = 18;
			Joints[18].GlobalInitPosition << 45.5f, 96.8537f, -17.9839f, 1.0f;
			Joints[18].ChildGlobalInitPosition << 48.758f, 113.572f, -29.5418f, 1.0f;
			Joints[18].parent_joint_index = 17;

			Joints[18].params_length = 1;
			Joints[18].params_index = new int[1]; Joints[18].params_type = new int[1];
			Joints[18].params_index[0] = 24; Joints[18].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[19].joint_name = "PinkeyTop";
			Joints[19].joint_index = 19;
			Joints[19].GlobalInitPosition << 48.758f, 113.572f, -29.5418f, 1.0f;
			Joints[19].ChildGlobalInitPosition << 50.609f, 131.057f, -42.4748f, 1.0f;
			Joints[19].parent_joint_index = 18;

			Joints[19].params_length = 1;
			Joints[19].params_index = new int[1]; Joints[19].params_type = new int[1];
			Joints[19].params_index[0] = 25; Joints[19].params_type[0] = dof_type(y_axis_rotate);
		}

		{
			Joints[20].joint_name = "PinkeySite";
			Joints[20].joint_index = 20;
			Joints[20].GlobalInitPosition << 50.609f, 131.057f, -42.4748f, 1.0f;
			Joints[20].HasChild = false;
			Joints[20].parent_joint_index = 19;

			Joints[20].params_length = 0;
			Joints[20].params_index = NULL; Joints[20].params_type = NULL;
		}
	}

	GlobalPosition << 0, 0, 0, 0;

	load_vertices(".\\model\\Vertex_new.txt");
	load_faces(".\\model\\Faces.txt");
	load_weight(".\\model\\Weight.txt");

	//Params对应关系
	//       0       ------>    wrist_T_x    //全局平移
	//       1       ------>    wrist_T_y    //全局平移
	//       2       ------>    wrist_T_z    //全局平移
	//       3       ------>    wrist_R_x
	//       4       ------>    wrist_R_y
	//       5       ------>    wrist_R_z
	//       6       ------>    Thumb_Low_R_y
	//       7       ------>    Thumb_Low_R_z
	//       8       ------>    Thumb_mid_R_z    //这里注意了，是z不是y了
	//       9       ------>    Thumb_top_R_z    //这里注意了，是z不是y了
	//       10      ------>    Index_Low_R_y
	//       11      ------>    Index_Low_R_z
	//       12      ------>    Index_mid_R_y
	//       13      ------>    Index_top_R_y
	//       14      ------>    Middle_Low_R_y
	//       15      ------>    Middle_Low_R_z
	//       16      ------>    Middle_mid_R_y
	//       17      ------>    Middle_top_R_y
	//       18      ------>    Ring_Low_R_y
	//       19      ------>    Ring_Low_R_z
	//       20      ------>    Ring_mid_R_y
	//       21      ------>    Ring_top_R_y
	//       22      ------>    Pinkey_Low_R_y
	//       23      ------>    Pinkey_Low_R_z
	//       24      ------>    Pinkey_mid_R_y
	//       25      ------>    Pinkey_top_R_y
	NumberofParams = 26;
	Params = new float[26]();
	ParamsUpperBound = new int[26]();
	ParamsLowerBound = new int[26]();


	//Scale对应关系
	//       0      ------>    整体变长
	//       1      ------>    整体变宽
	//       2      ------>    整体变厚
	Hand_scale << 1, 1, 1;

	//Jacobain related 
	Joints_jacobian = Eigen::MatrixXf::Zero(NumofJoints * 3, NumberofParams);

	compute_local_coordinate();
	compute_parent_child_transform();
	Updata(Params);

}

void HandModel::set_one_rotation(const Pose& pose, int index)
{
	Eigen::MatrixXf x = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf y = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf z = Eigen::MatrixXf::Identity(4, 4);

	float cx = cos(pose.x / 180 * PI);
	float sx = sin(pose.x / 180 * PI);

	float cy = cos(pose.y / 180 * PI);
	float sy = sin(pose.y / 180 * PI);

	float cz = cos(pose.z / 180 * PI);
	float sz = sin(pose.z / 180 * PI);

	x(1, 1) = cx; x(2, 2) = cx;
	x(1, 2) = -sx; x(2, 1) = sx;

	y(0, 0) = cy; y(0, 2) = sy;
	y(2, 0) = -sy; y(2, 2) = cy;

	z(0, 0) = cz; z(1, 1) = cz;
	z(0, 1) = -sz; z(1, 0) = sz;

	if (index == 0)
	{
		Joints[index].rotation = y*x*z;
	}
	else
	{
		if (index == 1)
		{
			Joints[index].rotation = z*x*y;
		}
		else
		{
			Joints[index].rotation = x*y*z;
		}
	}

}

void HandModel::compute_local_coordinate() {

	float axisx[3] = { 0.0f,0.0f,0.0f };
	float axisy[3] = { 0.0f,0.0f,0.0f };
	float axisz[3] = { 0.0f,0.0f,1.0f };

	for (int i = 0; i < NumofJoints; ++i)
	{
		if (Joints[i].HasChild)
		{
			
			axisz[0] = 0.0f; axisz[1] = 0.0f; axisz[2] = 1.0f;

			float position[3] = { Joints[i].GlobalInitPosition(0) ,Joints[i].GlobalInitPosition(1) ,Joints[i].GlobalInitPosition(2) };

			axisx[0] = Joints[i].ChildGlobalInitPosition(0) - Joints[i].GlobalInitPosition(0);
			axisx[1] = Joints[i].ChildGlobalInitPosition(1) - Joints[i].GlobalInitPosition(1);
			axisx[2] = Joints[i].ChildGlobalInitPosition(2) - Joints[i].GlobalInitPosition(2);

			normalize(axisx);
			cross_product(axisx, axisz, axisy);
			normalize(axisy);
			cross_product(axisx, axisy, axisz);
			normalize(axisz);

			Joints[i].local = Eigen::MatrixXf::Zero(4, 4);

			Joints[i].local(0, 0) = axisx[0]; Joints[i].local(1, 0) = axisx[1]; Joints[i].local(2, 0) = axisx[2];
			Joints[i].local(0, 1) = axisy[0]; Joints[i].local(1, 1) = axisy[1]; Joints[i].local(2, 1) = axisy[2];
			Joints[i].local(0, 2) = axisz[0]; Joints[i].local(1, 2) = axisz[1]; Joints[i].local(2, 2) = axisz[2];
			Joints[i].local(0, 3) = position[0]; Joints[i].local(1, 3) = position[1]; Joints[i].local(2, 3) = position[2];
			Joints[i].local(3, 3) = 1.0;
		}
		else
		{
			float position[3] = { Joints[i].GlobalInitPosition(0) ,Joints[i].GlobalInitPosition(1) ,Joints[i].GlobalInitPosition(2) };
			int parent_index = Joints[i].parent_joint_index;
			Joints[i].local = Joints[parent_index].local;
			Joints[i].local(0, 3) = position[0]; Joints[i].local(1, 3) = position[1]; Joints[i].local(2, 3) = position[2];
		}
	}
}

void HandModel::compute_parent_child_transform()
{
	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].trans = Joints[parent_joint_index].local.inverse()*Joints[i].local;
		}
		else
		{
			Joints[i].trans = Joints[i].local;
		}
	}
}

void HandModel::compute_rotation_matrix(float* params)
{

	//Params对应关系
	//       0       ------>    wrist_T_x    //全局平移
	//       1       ------>    wrist_T_y    //全局平移
	//       2       ------>    wrist_T_z    //全局平移
	//       3       ------>    wrist_R_x
	//       4       ------>    wrist_R_y
	//       5       ------>    wrist_R_z
	//       6       ------>    Thumb_Low_R_y
	//       7       ------>    Thumb_Low_R_z
	//       8       ------>    Thumb_mid_R_z    //这里注意了，是z不是y了
	//       9       ------>    Thumb_top_R_z    //这里注意了，是z不是y了
	//       10      ------>    Index_Low_R_y
	//       11      ------>    Index_Low_R_z
	//       12      ------>    Index_mid_R_y
	//       13      ------>    Index_top_R_y
	//       14      ------>    Middle_Low_R_y
	//       15      ------>    Middle_Low_R_z
	//       16      ------>    Middle_mid_R_y
	//       17      ------>    Middle_top_R_y
	//       18      ------>    Ring_Low_R_y
	//       19      ------>    Ring_Low_R_z
	//       20      ------>    Ring_mid_R_y
	//       21      ------>    Ring_top_R_y
	//       22      ------>    Pinkey_Low_R_y
	//       23      ------>    Pinkey_Low_R_z
	//       24      ------>    Pinkey_mid_R_y
	//       25      ------>    Pinkey_top_R_y

	GlobalPosition(0) = params[0];
	GlobalPosition(1) = params[1];
	GlobalPosition(2) = params[2];
	GlobalPosition(3) = 1;

	Pose p_wrist(params[3], params[4], params[5]);
	set_one_rotation(p_wrist, 0);

	//thumb
	Pose p_thumb_lower(0, params[6], params[7]);
	Pose p_thumb_middle(0, 0, params[8]);    //这里注意了，是z不是y了
	Pose p_thumb_top(0, 0, params[9]);      //这里注意了，是z不是y了
	set_one_rotation(p_thumb_lower, 1);
	set_one_rotation(p_thumb_middle, 2);
	set_one_rotation(p_thumb_top, 3);

	//index
	Pose p_pinkey_lower(0, params[10], params[11]);
	Pose p_pinkey_middle(0, params[12], 0);
	Pose p_pinkey_top(0, params[13], 0);
	set_one_rotation(p_pinkey_lower, 5);
	set_one_rotation(p_pinkey_middle, 6);
	set_one_rotation(p_pinkey_top, 7);

	//middle
	Pose p_ring_lower(0, params[14], params[15]);
	Pose p_ring_middle(0, params[16], 0);
	Pose p_ring_top(0, params[17], 0);
	set_one_rotation(p_ring_lower, 9);
	set_one_rotation(p_ring_middle, 10);
	set_one_rotation(p_ring_top, 11);

	//ring
	Pose p_middle_lower(0, params[18], params[19]);
	Pose p_middle_middle(0, params[20], 0);
	Pose p_middle_top(0, params[21], 0);
	set_one_rotation(p_middle_lower, 13);
	set_one_rotation(p_middle_middle, 14);
	set_one_rotation(p_middle_top, 15);

	//pinkey
	Pose p_index_lower(0, params[22], params[23]);
	Pose p_index_middle(0, params[24], 0);
	Pose p_index_top(0, params[25], 0);
	set_one_rotation(p_index_lower, 17);
	set_one_rotation(p_index_middle, 18);
	set_one_rotation(p_index_top, 19);
}

void HandModel::compute_global_matrix()
{
	//这里的Scaling是在Wrist局部坐标系下进行缩放，然后再将缩放后的坐标转换到世界坐标系下，然后在世界坐标系中进行Params[0~2]的平移
	Eigen::Matrix<float, 4, 4> Scaling = Eigen::MatrixXf::Zero(4, 4);
	Scaling(0, 0) = Hand_scale(0);
	Scaling(1, 1) = Hand_scale(1);
	Scaling(2, 2) = Hand_scale(2);
	Scaling(3, 3) = 1;

	Joints[0].global = Joints[0].local*Joints[0].rotation*Scaling;

	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].global = Joints[parent_joint_index].global*Joints[i].trans*Joints[i].rotation;
		}
	}
}

void HandModel::Updata_Joints_Axis()
{
	//updata joints
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joints[i].CorrespondingPosition << Joints[i].global*Joints[i].local.inverse()*Joints[i].GlobalInitPosition + GlobalPosition;
	}

	//updata axis
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joints[i].CorrespondingAxis[0] << Joints[i].global*Joints[i].dof_axis[0] + GlobalPosition;
		Joints[i].CorrespondingAxis[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition;
		Joints[i].CorrespondingAxis[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition;
	}
}

void HandModel::Updata_Vertics()
{
	Eigen::MatrixXf t = Eigen::MatrixXf::Zero(4, NumofVertices);
	Eigen::MatrixXf x = Eigen::MatrixXf::Ones(4, NumofVertices);
	x.block(0, 0, 3, NumofVertices) = Vectices.block(0, 0, NumofVertices, 3).transpose();

	Eigen::MatrixXf y;
	Eigen::MatrixXf y0;
	Eigen::MatrixXf z;

	for (int i = 0; i < NumofJoints; ++i) {
		y = Weights.block(0, i, NumofVertices, 1);// 在所有顶点 对于 该关节点的weight
		y0 = y.replicate(1, 4);    //分别是行重复1遍，列重复4遍，结果为（num_vertices_，4）这么大小的矩阵
		z = Joints[i].global * Joints[i].local.inverse() * x;
		t = t + z.cwiseProduct(y0.transpose());
	}
	vertices_update_ = t.transpose();

	for (int i = 0; i < vertices_update_.rows(); ++i) {
		vertices_update_(i, 0) += GlobalPosition(0);
		vertices_update_(i, 1) += GlobalPosition(1);
		vertices_update_(i, 2) += GlobalPosition(2);
	}
}

void HandModel::Updata(float* params)
{

	compute_rotation_matrix(params);
	compute_global_matrix();
	Updata_Joints_Axis();

	Updata_Vertics();        //经过测试最耗时 ： 共需要8ms左右

	Compute_normal_And_visibel_vertices();    //耗时需要1ms左右

	Joint_matrix = Eigen::MatrixXf::Zero(NumofJoints, 3);
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joint_matrix(i, 0) = Joints[i].CorrespondingPosition(0);
		Joint_matrix(i, 1) = Joints[i].CorrespondingPosition(1);
		Joint_matrix(i, 2) = Joints[i].CorrespondingPosition(2);
	}

}

void HandModel::Compute_normal_And_visibel_vertices()
{
	Visible_vertices.clear();
	Visible_vertices_index.clear();
	Face_normal.clear();
	Vertices_normal.setZero();
	Vector3f A, B, C, BA, BC;
	for (int i = 0; i < NumofFaces; ++i)
	{
		//这里我假设，如果假设错了，那么叉乘时候，就BC*BA变成BA*BC
		//            A
		//          /  \
		//         B — C
		A << vertices_update_(FaceIndex(i, 0), 0), vertices_update_(FaceIndex(i, 0), 1), vertices_update_(FaceIndex(i, 0), 2);
		B << vertices_update_(FaceIndex(i, 1), 0), vertices_update_(FaceIndex(i, 1), 1), vertices_update_(FaceIndex(i, 1), 2);
		C << vertices_update_(FaceIndex(i, 2), 0), vertices_update_(FaceIndex(i, 2), 1), vertices_update_(FaceIndex(i, 2), 2);

		BC << C - B;
		BA << A - B;

		Vector3f nom(BC.cross(BA));

		nom.normalize();
		Face_normal.push_back(nom);

		Vertices_normal(FaceIndex(i, 0), 0) += nom(0);
		Vertices_normal(FaceIndex(i, 1), 0) += nom(0);
		Vertices_normal(FaceIndex(i, 2), 0) += nom(0);

		Vertices_normal(FaceIndex(i, 0), 1) += nom(1);
		Vertices_normal(FaceIndex(i, 1), 1) += nom(1);
		Vertices_normal(FaceIndex(i, 2), 1) += nom(1);

		Vertices_normal(FaceIndex(i, 0), 2) += nom(2);
		Vertices_normal(FaceIndex(i, 1), 2) += nom(2);
		Vertices_normal(FaceIndex(i, 2), 2) += nom(2);

	}

	for (int i = 0; i < Vertices_normal.rows(); ++i)
	{
		Vertices_normal.row(i).normalize();

		if (-(Vertices_normal(i, 2)) >= 0)
		{
			Vector3f visible_v(vertices_update_(i, 0), vertices_update_(i, 1), vertices_update_(i, 2));
			Visible_vertices.push_back(visible_v);
			Visible_vertices_index.push_back(i);
		}
	}

	//cout << "visible vertices find done!" << endl;
}


//=================Jacobain    related    function==============================
void HandModel::Updata_joints_Jacobian()
{
	Joints_jacobian.setZero();

	for (int i = 1; i < NumofJoints; ++i)
	{
		Joints_jacobian.block(i * 3, 0, 3, NumberofParams) = Compute_one_Joint_Jacobian(i);
	}
}

Eigen::MatrixXf HandModel::Compute_one_Joint_Jacobian(int index)
{
	float omiga = 3.141592f / 180.0f;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);
	
	int current_indx = index;
	Eigen::Vector3f current_joint_position(Joints[index].CorrespondingPosition(0), Joints[index].CorrespondingPosition(1), Joints[index].CorrespondingPosition(2));
	
	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	while (current_indx >= 0)
	{
		axis_base_position<<Joints[current_indx].CorrespondingPosition(0), Joints[current_indx].CorrespondingPosition(1), Joints[current_indx].CorrespondingPosition(2);

		x_axis_position<<Joints[current_indx].CorrespondingAxis[0](0), Joints[current_indx].CorrespondingAxis[0](1), Joints[current_indx].CorrespondingAxis[0](2);
		y_axis_position<<Joints[current_indx].CorrespondingAxis[1](0), Joints[current_indx].CorrespondingAxis[1](1), Joints[current_indx].CorrespondingAxis[1](2);
		z_axis_position<<Joints[current_indx].CorrespondingAxis[2](0), Joints[current_indx].CorrespondingAxis[2](1), Joints[current_indx].CorrespondingAxis[2](2);

		
		w_x << (x_axis_position - axis_base_position);
		w_y << (y_axis_position - axis_base_position);
		w_z << (z_axis_position - axis_base_position);

		w_x.normalize();
		w_y.normalize();
		w_z.normalize();

		S << (current_joint_position - axis_base_position);

		int params_len = Joints[current_indx].params_length;
		for (int idx = 0; idx < params_len; idx++)
		{
			int params_idx = Joints[current_indx].params_index[idx];

			switch (Joints[current_indx].params_type[idx])
			{
			case dof_type(x_axis_rotate): {
				result << omiga*w_x.cross(S);
				Joints_jacobian(0, params_idx) += result(0);
				Joints_jacobian(1, params_idx) += result(1);
				Joints_jacobian(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_rotate): {
				result << omiga*w_y.cross(S);
				Joints_jacobian(0, params_idx) += result(0);
				Joints_jacobian(1, params_idx) += result(1);
				Joints_jacobian(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_rotate): {
				result << omiga*w_z.cross(S);
				Joints_jacobian(0, params_idx) += result(0);
				Joints_jacobian(1, params_idx) += result(1);
				Joints_jacobian(2, params_idx) += result(2);
				break;
			}
			case dof_type(x_axis_trans): {
				result << 1, 0, 0;
				Joints_jacobian(0, params_idx) += result(0);
				Joints_jacobian(1, params_idx) += result(1);
				Joints_jacobian(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_trans): {
				result << 0, 1, 0;
				Joints_jacobian(0, params_idx) += result(0);
				Joints_jacobian(1, params_idx) += result(1);
				Joints_jacobian(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_trans): {
				result << 0, 0, 1;
				Joints_jacobian(0, params_idx) += result(0);
				Joints_jacobian(1, params_idx) += result(1);
				Joints_jacobian(2, params_idx) += result(2);
				break;
			}
			}

		}

		current_indx = Joints[current_indx].parent_joint_index;
	}

	return Jacobain_;
}
