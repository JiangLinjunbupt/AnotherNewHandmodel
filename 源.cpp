#include "GL/freeglut.h"         //OpenGL的头文件Includ顺序会导致出现编译错误
#include"HandModel.h"
#include <tchar.h>
#include<ctime>

struct Control {
	int x;
	int y;
	bool mouse_click;
	GLfloat rotx;
	GLfloat roty;
	double gx;
	double gy;
	double gz;

	Control() :x(0), y(0), rotx(0.0), roty(0.0), mouse_click(false),
		gx(0), gy(0), gz(0) {

	}
};
Control control;
HandModel *handmodel = new HandModel();
bool show_mesh = true;
clock_t  Begin, End;
double duration;

//共享内存的相关定义
HANDLE hMapFile;
LPCTSTR pBuf;
#define BUF_SIZE 1024
TCHAR szName[] = TEXT("Global\\MyFileMappingObject");    //指向同一块共享内存的名字
float *GetSharedMemeryPtr;
float *GetGloveData = new float[26];


#pragma region OpenGL
void light() {
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	// 定义太阳光源，它是一种白色的光源  
	GLfloat sun_light_position[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat sun_light_ambient[] = { 0.25f, 0.25f, 0.15f, 1.0f };
	GLfloat sun_light_diffuse[] = { 0.7f, 0.7f, 0.55f, 1.0f };
	GLfloat sun_light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position); //指定第0号光源的位置   
	glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient); //GL_AMBIENT表示各种光线照射到该材质上，  
														 //经过很多次反射后最终遗留在环境中的光线强度（颜色）  
	glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse); //漫反射后~~  
	glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);//镜面反射后~~~  

	glEnable(GL_LIGHT0); //使用第0号光照   
}
/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {

	switch (key) {
	case  27:   // ESC
		exit(0);
	case 'm':
		show_mesh = !show_mesh;
		break;
	}
}

/* reshaped window */
void reshape(int width, int height) {

	GLfloat fieldOfView = 90.0f;
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
	control.mouse_click = 1;
	control.x = x;
	control.y = y;
}

void mouseMotion(int x, int y) {
	control.rotx = (x - control.x)*0.05f;
	control.roty = (y - control.y)*0.05f;

	//cout<< control.rotx <<" " << control.roty << endl;
	glutPostRedisplay();
}

/* render the scene */

void draw_mesh()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	{
		glColor3f(0.4, 0.8, 0.3);
		glBegin(GL_TRIANGLES);
		for (int i = 0; i < handmodel->NumofFaces; ++i)
		{

			glNormal3f(handmodel->Face_normal[i](0), handmodel->Face_normal[i](1), handmodel->Face_normal[i](2));

			glVertex3f(handmodel->vertices_update_(handmodel->FaceIndex(i, 0), 0), handmodel->vertices_update_(handmodel->FaceIndex(i, 0), 1), handmodel->vertices_update_(handmodel->FaceIndex(i, 0), 2));
			glVertex3f(handmodel->vertices_update_(handmodel->FaceIndex(i, 1), 0), handmodel->vertices_update_(handmodel->FaceIndex(i, 1), 1), handmodel->vertices_update_(handmodel->FaceIndex(i, 1), 2));
			glVertex3f(handmodel->vertices_update_(handmodel->FaceIndex(i, 2), 0), handmodel->vertices_update_(handmodel->FaceIndex(i, 2), 1), handmodel->vertices_update_(handmodel->FaceIndex(i, 2), 2));
		}
		glEnd();
	}
}

void draw_vertex()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glBegin(GL_POINTS);
	glColor3d(1.0, 1.0, 0.0);
	for (int i = 0; i < handmodel->NumofVertices; ++i) {
		glVertex3d(handmodel->vertices_update_(i, 0), handmodel->vertices_update_(i, 1), handmodel->vertices_update_(i, 2));
	}
	glEnd();
}

void draw_skeleton()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	for (int i = 0; i < handmodel->NumofJoints; ++i) {
		//画点开始 
		glColor3f(1.0, 0.0, 0.0);
		glPushMatrix();
		glTranslatef(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
		glutSolidSphere(3, 31, 10);
		glPopMatrix();
		//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。
		int parent_joint_index = handmodel->Joints[i].parent_joint_index;
		//画线开始  //不画wrist到arm的那条线
		if (parent_joint_index != -1) {
			glLineWidth(5);
			glColor3f(0.0, 1.0, 0);
			glBegin(GL_LINES);
			glVertex3f(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
			glVertex3f(handmodel->Joints[parent_joint_index].CorrespondingPosition(0), handmodel->Joints[parent_joint_index].CorrespondingPosition(1), handmodel->Joints[parent_joint_index].CorrespondingPosition(2));
			glEnd();
		}
	}
}

void draw_Coordinate()
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	//x
	glLineWidth(5);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(100, 0, 0);
	glEnd();

	//y
	glLineWidth(5);
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 100, 0);
	glEnd();

	//z
	glLineWidth(5);
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 100);
	glEnd();
}

void draw_joint_local_coordinate(int index)
{
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	int scale = 30;

	Vector4 x = handmodel->Joints[index].CorrespondingAxis[0] - handmodel->Joints[index].CorrespondingPosition;
	glLineWidth(2);
	glColor3f(1.0, 0.0, 0);
	glBegin(GL_LINES);
	glVertex3f(handmodel->Joints[index].CorrespondingPosition(0), handmodel->Joints[index].CorrespondingPosition(1), handmodel->Joints[index].CorrespondingPosition(2));
	glVertex3f(handmodel->Joints[index].CorrespondingPosition(0)+ scale*x(0), handmodel->Joints[index].CorrespondingPosition(1) + scale * x(1), handmodel->Joints[index].CorrespondingPosition(2) + scale * x(2));
	glEnd();


	Vector4 y = handmodel->Joints[index].CorrespondingAxis[1] - handmodel->Joints[index].CorrespondingPosition;
	glLineWidth(2);
	glColor3f(0.0, 1.0, 0);
	glBegin(GL_LINES);
	glVertex3f(handmodel->Joints[index].CorrespondingPosition(0), handmodel->Joints[index].CorrespondingPosition(1), handmodel->Joints[index].CorrespondingPosition(2));
	glVertex3f(handmodel->Joints[index].CorrespondingPosition(0) + scale * y(0), handmodel->Joints[index].CorrespondingPosition(1) + scale * y(1), handmodel->Joints[index].CorrespondingPosition(2) + scale * y(2));
	glEnd();

	Vector4 z = handmodel->Joints[index].CorrespondingAxis[2] - handmodel->Joints[index].CorrespondingPosition;
	glLineWidth(2);
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(handmodel->Joints[index].CorrespondingPosition(0), handmodel->Joints[index].CorrespondingPosition(1), handmodel->Joints[index].CorrespondingPosition(2));
	glVertex3f(handmodel->Joints[index].CorrespondingPosition(0) + scale * z(0), handmodel->Joints[index].CorrespondingPosition(1) + scale * z(1), handmodel->Joints[index].CorrespondingPosition(2) + scale * z(2));
	glEnd();
}

void draw() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	gluPerspective(180, 1.5, -1000, 1000);
	glLoadIdentity();
	control.gx = handmodel->GlobalPosition(0);
	control.gy = handmodel->GlobalPosition(1);
	control.gz = handmodel->GlobalPosition(2);
	double r = 210;
	double x = r*sin(control.roty)*cos(control.rotx);
	double y = r*sin(control.roty)*sin(control.rotx);
	double z = r*cos(control.roty);
	//cout<< x <<" "<< y <<" " << z<<endl;
	gluLookAt(x + control.gx, y + control.gy, z + control.gz, control.gx, control.gy, control.gz, 0.0, 1.0, 0.0);//个人理解最开始是看向-z的，之后的角度是在global中心上叠加的，所以要加

	if(show_mesh)  draw_mesh();
	//draw_vertex();
	draw_skeleton();
	draw_Coordinate();
	for(int i = 0;i<21;++i)  	draw_joint_local_coordinate(i);


	//glPointSize(2);
	//glBegin(GL_POINTS);
	//glColor3d(1.0, 0.0, 0.0);
	//for (int i = 0; i < handmodel->Visible_vertices.size(); i++) {
	//	glVertex3d(handmodel->Visible_vertices[i](0), handmodel->Visible_vertices[i](1), handmodel->Visible_vertices[i](2));
	//}
	//glEnd();



	glFlush();
	glutSwapBuffers();
}

/* executed when program is idle */
void idle() {

	Begin = clock();//开始计时

	for (int i = 0; i < 26; ++i)
	{
		GetGloveData[i] = GetSharedMemeryPtr[i];
	}

	//GetGloveData[3] = -180;
	//GetGloveData[4] = -0;
	//GetGloveData[5] = -0;
	//GetGloveData[6] = 20;
	//GetGloveData[7] = 35;

	handmodel->Updata(GetGloveData);


	End = clock();//结束计时
	duration = double(End - Begin) / CLK_TCK;//duration就是运行函数所打的

	//std::cout << "time is : " << duration * 1000 << std::endl;

	glutPostRedisplay();
}

/* initialize OpenGL settings */
void initGL(int width, int height) {

	reshape(width, height);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	light();
}

#pragma endregion 

int main(int argc, char** argv)
{

#pragma region SharedMemery
	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security
		PAGE_READWRITE,          // read/write access
		0,                       // maximum object size (high-order DWORD)
		BUF_SIZE,                // maximum object size (low-order DWORD)
		szName);                 // name of mapping object

	if (hMapFile == NULL)
	{
		_tprintf(TEXT("Could not create file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}
	pBuf = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,
		0,
		BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"),
			GetLastError());

		CloseHandle(hMapFile);

		return 1;
	}

	GetSharedMemeryPtr = (float*)pBuf;
#pragma endregion SharedMemery


#pragma region OpenGL
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Interactron");

	// register glut call backs
	glutKeyboardFunc(keyboardDown);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(reshape);
	glutDisplayFunc(draw);
	glutIdleFunc(idle);
	glutIgnoreKeyRepeat(true); // ignore keys held down
	initGL(800, 600);

	glutMainLoop();

#pragma endregion

	return 0;
}