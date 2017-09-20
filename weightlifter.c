/*
  Weightlifter, by Allen Han
  SCU Spring 2016, COEN 290, Assignment 2

  [COMPLETE] must be able to select pick any object (any part of the robot or the sport object);
             must be able to rotate the selected object left or right (left rotation with left 
             click and right rotation with right click, with the mouse)
  [COMPLETE] must be able to translate the selected object
  [COMPLETE] perform playback and record with the following pop-up menus or pre-defined keys

  Key inputs: (same as Robot_Arm_with_Playback)
     b - begin recording
     e - end recording
     s - save to playback file
     l - load playback file and reset transformation
     p - playback the recording
     r - reset transformation

  Picking Instructions:
     Select appendage as necessary.
     Rotate about axis with either left click or right click.
     Head and torso, besides rotation with mouse click, have additional degree of freedom and can
       be rotated with the scrollwheel.
     For translation of the figure, pick the figure with the middle mouse click, then drag and 
       release.
*/

#include <GL/glut.h>
#include <stdio.h>

#define TREE_NULL 0

#define ABD_VERT_RADIUS 1.0
#define ABD_HORI_RADIUS 1.0
#define CHE_VERT_RADIUS 2.0
#define CHE_HORI_RADIUS 1.5

#define TORSO_HEIGHT 5.0
#define TORSO_RADIUS 1.0

#define UPPER_ARM_HEIGHT 3.0
#define UPPER_ARM_RADIUS  0.5

#define LOWER_ARM_HEIGHT 2.0
#define LOWER_ARM_RADIUS  0.5

#define UPPER_LEG_HEIGHT 3.0
#define UPPER_LEG_RADIUS  0.5

#define LOWER_LEG_HEIGHT 2.5
#define LOWER_LEG_RADIUS  0.5

#define HEAD_HEIGHT 1.5
#define HEAD_RADIUS 1.0

#define WEIGHT_BAR_RADIUS 0.25
#define WEIGHT_BAR_LENGTH 2.5
#define WEIGHT_RADIUS 2.0
#define WEIGHT_LENGTH 0.25

void processHits(GLint hits, GLuint buffer[]);

typedef struct treenode{
  GLint id_number;
  GLfloat homogeneous_m[16];
  void (*drawing_f)();
  void (*setting_f)();
  struct treenode *sibling;
  struct treenode *child;
}treenode; 

typedef struct event{
  int obj_id;
  GLfloat event_data1;
  GLfloat event_data2;
}event;

void head_f(GLenum mode, treenode* root);
void torso_f(GLenum mode, treenode* root);
void left_upper_arm_f(GLenum mode, treenode* root);
void right_upper_arm_f(GLenum mode, treenode* root);
void left_lower_arm_f(GLenum mode, treenode* root);
void right_lower_arm_f(GLenum mode, treenode* root);
void left_upper_leg_f(GLenum mode, treenode* root);
void right_upper_leg_f(GLenum mode, treenode* root);
void left_lower_leg_f(GLenum mode, treenode* root);
void right_lower_leg_f(GLenum mode, treenode* root);
void weight_bar_1_inside_f(GLenum mode, treenode* root);
void weight_bar_1_outside_f(GLenum mode, treenode* root);
void weight_1_f(GLenum mode, treenode* root);
void weight_bar_2_inside_f(GLenum mode, treenode* root);
void weight_bar_2_outside_f(GLenum mode, treenode* root);
void weight_2_f(GLenum mode, treenode* root);

void set_t();
void set_h();
void set_lua();
void set_rua();
void set_lul();
void set_rul();
void set_lla();
void set_rla();
void set_lll();
void set_rll();
void set_wb1i();
void set_wb1o();
void set_w1();
void set_wb2i();
void set_wb2o();
void set_w2();

typedef float point[3];

typedef treenode* t_ptr;

static GLfloat theta[7][2] = { {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} };

static GLint selectionName = 1;

GLUquadricObj *abd_q, *che_q, *h_q, *lua_q, *lla_q, 
                                    *rua_q, *rla_q, *lll_q, *rll_q, 
              *rul_q, *lul_q, *wb1i_q, *wb1o_q, *w1_q, *wb2i_q, *wb2o_q, *w2_q;

int winWidth = 500;
int winHeight = 500;

double size=1.0;

int currentButton;

treenode torso_node, head_node,
         lua_node, rua_node, lll_node, rll_node,
         lla_node, rla_node, rul_node, lul_node, 
         wb1i_node, wb1o_node, w1_node,
         wb2i_node, wb2o_node, w2_node;

int drawObjectsIndex = 0;

GLfloat origin[2] = {0.0, 0.0};
GLfloat setPoint[2] = {0.0, 0.0};

#define MAXEVENTS 8000

event event_buffer[MAXEVENTS];
int event_ptr = 0;
int playback_ptr = 0;

int recordMode = 0;
int playbackMode = 0;

FILE *jFile = NULL;
char *fileName = "record.txt";

void traverse(GLenum mode, treenode* root)
{
  if(root==TREE_NULL) return;
  glPushMatrix();
  root->drawing_f(mode, root);
  if(root->child!=TREE_NULL) traverse(mode, root->child);
  glPopMatrix();
  if(root->sibling!=TREE_NULL) traverse(mode, root->sibling);
}

void torso_f(GLenum mode, treenode* root)
{
  //printf("torso node\n");
  if (mode == GL_SELECT) glLoadName(torso_node.id_number);
  if (selectionName == 1)
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();

  glMultMatrixf(root->homogeneous_m);
  
  glPushMatrix();
  glScalef(CHE_HORI_RADIUS/CHE_VERT_RADIUS,1.0,CHE_HORI_RADIUS/CHE_VERT_RADIUS);
  glTranslatef(0.0,CHE_VERT_RADIUS+ABD_VERT_RADIUS,0.0);
  gluSphere(che_q,CHE_VERT_RADIUS,10,10);
  glPopMatrix();

  glPushMatrix();
  glScalef(ABD_HORI_RADIUS/ABD_VERT_RADIUS,1.0,ABD_HORI_RADIUS/ABD_VERT_RADIUS);
  gluSphere(abd_q,ABD_VERT_RADIUS,10,10);
  glPopMatrix();
  //glRotatef(-90.0, 1.0, 0.0, 0.0);
  //gluCylinder(t_q,TORSO_RADIUS, TORSO_RADIUS, TORSO_HEIGHT,10,10);
}

void head_f(GLenum mode, treenode* root)
{
  //printf("head node\n");
  if (mode == GL_SELECT) glLoadName(head_node.id_number);
  if (selectionName == 2 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glTranslatef(0.0, 0.5*HEAD_HEIGHT,0.0);
  glScalef(HEAD_RADIUS, HEAD_HEIGHT, HEAD_RADIUS);
  gluSphere(h_q,1.0,10,10);
  glPopMatrix();
}

void left_upper_arm_f(GLenum mode, treenode* root)
{
  //printf("left upper arm node\n");
  if (mode == GL_SELECT) glLoadName(lua_node.id_number);
  if (selectionName == 3 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 1.0, 0.0, 0.0);
  gluCylinder(lua_q,UPPER_ARM_RADIUS, UPPER_ARM_RADIUS, UPPER_ARM_HEIGHT,10,10);
  glPopMatrix();
}

void left_lower_arm_f(GLenum mode, treenode* root)
{
  //printf("left lower arm node\n");
  if (mode == GL_SELECT) glLoadName(lla_node.id_number);
  if (selectionName == 4 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 1.0, 0.0, 0.0);
  gluCylinder(lla_q,LOWER_ARM_RADIUS, LOWER_ARM_RADIUS, LOWER_ARM_HEIGHT,10,10);
  glPopMatrix();
}

void right_upper_arm_f(GLenum mode, treenode* root)
{
  //printf("right upper arm node\n");
  if (mode == GL_SELECT) glLoadName(rua_node.id_number);
  if (selectionName == 3 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 1.0, 0.0, 0.0);
  gluCylinder(rua_q,UPPER_ARM_RADIUS, UPPER_ARM_RADIUS, UPPER_ARM_HEIGHT,10,10);
  glPopMatrix();
}

void right_lower_arm_f(GLenum mode, treenode* root)
{
  //printf("right lower arm node\n");
  if (mode == GL_SELECT) glLoadName(rla_node.id_number);
  if (selectionName == 4 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 1.0, 0.0, 0.0);
  gluCylinder(rla_q,LOWER_ARM_RADIUS, LOWER_ARM_RADIUS, LOWER_ARM_HEIGHT,10,10);
  glPopMatrix();
}

void left_upper_leg_f(GLenum mode, treenode* root)
{
  //printf("left upper leg node\n");
  if (mode == GL_SELECT) glLoadName(lul_node.id_number);
  if (selectionName == 5 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(lul_q,UPPER_LEG_RADIUS, UPPER_LEG_RADIUS, UPPER_LEG_HEIGHT,10,10);
  glPopMatrix();
}

void left_lower_leg_f(GLenum mode, treenode* root)
{
  //printf("left lower leg node\n");
  if (mode == GL_SELECT) glLoadName(lll_node.id_number);
  if (selectionName == 6 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(lll_q,LOWER_LEG_RADIUS, LOWER_LEG_RADIUS, LOWER_LEG_HEIGHT,10,10);
  glPopMatrix();
}

void right_upper_leg_f(GLenum mode, treenode* root)
{
  //printf("right upper leg node\n");
  if (mode == GL_SELECT) glLoadName(rul_node.id_number);
  if (selectionName == 5 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(rul_q,UPPER_LEG_RADIUS, UPPER_LEG_RADIUS, UPPER_LEG_HEIGHT,10,10);
  glPopMatrix();
}

void right_lower_leg_f(GLenum mode, treenode* root)
{
  //printf("right lower leg node\n");
  if (mode == GL_SELECT) glLoadName(rll_node.id_number);
  if (selectionName == 6 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(rll_q,LOWER_LEG_RADIUS, LOWER_LEG_RADIUS, LOWER_LEG_HEIGHT,10,10);
  glPopMatrix();
}

void weight_bar_1_inside_f(GLenum mode, treenode* root)
{
  //printf("weight bar node\n");
  if (mode == GL_SELECT) glLoadName(wb1i_node.id_number);
  if (selectionName == 7 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 0.0, 1.0, 0.0);
  gluCylinder(wb1i_q,WEIGHT_BAR_RADIUS, WEIGHT_BAR_RADIUS, WEIGHT_BAR_LENGTH,10,10);
  glPopMatrix();
}

void weight_bar_1_outside_f(GLenum mode, treenode* root)
{
  //printf("weight bar node\n");
  if (mode == GL_SELECT) glLoadName(wb1o_node.id_number);
  if (selectionName == 7 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 0.0, 1.0, 0.0);
  gluCylinder(wb1o_q,WEIGHT_BAR_RADIUS, WEIGHT_BAR_RADIUS, WEIGHT_BAR_LENGTH,10,10);
  glPopMatrix();
}

void weight_1_f(GLenum mode, treenode* root)
{
  //printf("weight bar node\n");
  if (mode == GL_SELECT) glLoadName(w1_node.id_number);
  if (selectionName == 7 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 0.0, 1.0, 0.0);
  gluCylinder(w1_q,WEIGHT_RADIUS, WEIGHT_RADIUS, WEIGHT_LENGTH,10,10);
  glPopMatrix();
}

void weight_bar_2_inside_f(GLenum mode, treenode* root)
{
  //printf("weight bar node\n");
  if (mode == GL_SELECT) glLoadName(wb2i_node.id_number);
  if (selectionName == 7 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(-90.0, 0.0, 1.0, 0.0);
  gluCylinder(wb2i_q,WEIGHT_BAR_RADIUS, WEIGHT_BAR_RADIUS, WEIGHT_BAR_LENGTH,10,10);
  glPopMatrix();
}

void weight_bar_2_outside_f(GLenum mode, treenode* root)
{
  //printf("weight bar node\n");
  if (mode == GL_SELECT) glLoadName(wb2o_node.id_number);
  if (selectionName == 7 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 0.0, 1.0, 0.0);
  gluCylinder(wb2o_q,WEIGHT_BAR_RADIUS, WEIGHT_BAR_RADIUS, WEIGHT_BAR_LENGTH,10,10);
  glPopMatrix();
}

void weight_2_f(GLenum mode, treenode* root)
{
  //printf("weight bar node\n");
  if (mode == GL_SELECT) glLoadName(w2_node.id_number);
  if (selectionName == 7 )
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  root->setting_f();
  glPopMatrix();
  glMultMatrixf(root->homogeneous_m);
  glPushMatrix();
  glRotatef(90.0, 0.0, 1.0, 0.0);
  gluCylinder(w2_q,WEIGHT_RADIUS, WEIGHT_RADIUS, WEIGHT_LENGTH,10,10);
  glPopMatrix();
}

void drawObjects(GLenum mode)
{
  drawObjectsIndex++;
  //printf("drawObjectsIndex is %d\n", drawObjectsIndex);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(origin[0], origin[1], 0);
  traverse(mode, &torso_node);
  glFlush();
  glutSwapBuffers();
}

void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (winWidth <= winHeight)
    glOrtho(-10.0, 10.0, -10.0 * (GLfloat) winHeight / (GLfloat) winWidth,
	    10.0 * (GLfloat) winHeight / (GLfloat) winWidth, -10.0, 10.0);
  else
    glOrtho(-10.0 * (GLfloat) winWidth / (GLfloat) winHeight,
	    10.0 * (GLfloat) winWidth / (GLfloat) winHeight, -10.0, 10.0, -10.0, 10.0);
  
  glMatrixMode(GL_MODELVIEW);
  drawObjects(GL_RENDER);
  glFlush();
}

#define SIZE 512

void mouse(int btn, int state, int x, int y)
{
  GLuint selectBuf[SIZE];
  GLint hits;
  GLint viewport[4];
  
  if(btn==GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {

    setPoint[0] = x;
    setPoint[1] = y;

    currentButton = GLUT_MIDDLE_BUTTON;
    glGetIntegerv(GL_VIEWPORT, viewport);

    glSelectBuffer(SIZE, selectBuf);
    glRenderMode(GL_SELECT);

    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    gluPickMatrix((GLdouble) x, (GLdouble) (viewport[3]-y), 5.0, 5.0, viewport);

    if(winWidth <= winHeight)
      glOrtho(-10.0, 10.0, -10.0 * (GLfloat) winHeight / (GLfloat) winWidth, 10.0 * (GLfloat) winHeight / (GLfloat) winWidth, -10.0, 10.0);
    else
      glOrtho(-10.0 * (GLfloat) winWidth / (GLfloat) winHeight, 10.0 * (GLfloat) winWidth / (GLfloat) winHeight, -10.0, 10.0, -10.0, 10.0);

    drawObjects(GL_SELECT);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glFlush();

    hits = glRenderMode(GL_RENDER);
    processHits(hits, selectBuf);

    glutPostRedisplay();
  } else if(btn==GLUT_LEFT_BUTTON && state==GLUT_DOWN){
    currentButton = GLUT_LEFT_BUTTON;
    theta[selectionName-1][0] += 5.0;
    //printf("angle is now %f.\n", theta[selectionName-1][0]);
    if(theta[selectionName-1][0] > 360.0) theta[selectionName-1][0] -= 360.0;
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = theta[selectionName-1][0];
      event_buffer[event_ptr].event_data2 = theta[selectionName-1][1];
      event_ptr++;
    }
    display();
  } else if(btn==GLUT_RIGHT_BUTTON && state==GLUT_DOWN){
    currentButton = GLUT_RIGHT_BUTTON;
    theta[selectionName-1][0] -= 5.0;
    //printf("angle is now %f.\n", theta[selectionName-1][0]);
    if(theta[selectionName-1][0] < 0.0) theta[selectionName-1][0] += 360.0;
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = theta[selectionName-1][0];
      event_buffer[event_ptr].event_data2 = theta[selectionName-1][1];
      event_ptr++;
    }
    display();
  } else if(btn==3 && state==GLUT_DOWN){
    currentButton = GLUT_MIDDLE_BUTTON;
    theta[selectionName-1][1] += 5.0;
    //printf("angle is now %f.\n", theta[selectionName-1][1]);
    if(theta[selectionName-1][1] > 360.0) theta[selectionName-1][1] -= 360.0;
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = theta[selectionName-1][0];
      event_buffer[event_ptr].event_data2 = theta[selectionName-1][1];
      event_ptr++;
    }
    display();
  } else if(btn==4 && state==GLUT_DOWN){
    currentButton = GLUT_MIDDLE_BUTTON;
    theta[selectionName-1][1] -= 5.0;
    //printf("angle is now %f.\n", theta[selectionName-1][1]);
    if(theta[selectionName-1][1] < 0.0) theta[selectionName-1][1] += 360.0;
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = theta[selectionName-1][0];
      event_buffer[event_ptr].event_data2 = theta[selectionName-1][1];
      event_ptr++;
    }
    display();
  }

}

void processHits(GLint hits, GLuint buffer[])
{
   unsigned int i, j;
   GLuint ii, jj, names, *ptr;

   printf ("hits = %d\n", hits);
   selectionName = 0;
   ptr = (GLuint *) buffer; 
   for (i = 0; i < hits; i++) {	/*  for each hit  */
     names = *ptr;
     ptr+=3;
     for (j = 0; j < names; j++) { /*  for each name */
       printf("object name: %d\n", *ptr);
       selectionName = *ptr;
       ptr++;
     }
   }
}

void myReshape(int w, int h)
{
  winHeight = h;
  winWidth = w;
  
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  if (w <= h) glOrtho(-10.0, 10.0, -10.0 * (GLfloat) h / (GLfloat) w,
                                    10.0 * (GLfloat) h / (GLfloat) w, -10.0, 10.0);
  else glOrtho(-10.0 * (GLfloat) w / (GLfloat) h,
	       10.0 * (GLfloat) w / (GLfloat) h, 0.0, 10.0, -10.0, 10.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void set_t(){
  // torso
  glLoadIdentity();
  glRotatef(theta[0][0], 0.0, 1.0, 0.0);
  glRotatef(theta[0][1], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,torso_node.homogeneous_m);
}  

void set_h(){
  // head
  glLoadIdentity();
  glTranslatef(0.0, TORSO_HEIGHT+0.5*HEAD_HEIGHT, 0.0);
  glRotatef(theta[1][0], 1.0, 0.0, 0.0);
  glRotatef(theta[1][1], 0.0, 1.0, 0.0);
  glTranslatef(0.0, -0.5*HEAD_HEIGHT, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,head_node.homogeneous_m);
}

void set_lua(){
  // left upper arm
  glLoadIdentity();
  glTranslatef(-(TORSO_RADIUS+UPPER_ARM_RADIUS), 0.9*TORSO_HEIGHT, 0.0);
  glRotatef(theta[2][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,lua_node.homogeneous_m);
}

void set_rua(){
  // right upper arm
  glLoadIdentity();
  glTranslatef(TORSO_RADIUS+UPPER_ARM_RADIUS, 0.9*TORSO_HEIGHT, 0.0);
  glRotatef(theta[2][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,rua_node.homogeneous_m);
}

void set_lla(){
  // left lower arm
  glLoadIdentity();
  glTranslatef(0.0, UPPER_ARM_HEIGHT, 0.0);
  glRotatef(theta[3][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,lla_node.homogeneous_m);
}

void set_rla(){
  // right lower arm
  glLoadIdentity();
  glTranslatef(0.0, UPPER_ARM_HEIGHT, 0.0);
  glRotatef(theta[3][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,rla_node.homogeneous_m);
}

void set_lul(){
  // left upper leg
  glLoadIdentity();
  glTranslatef(-(TORSO_RADIUS+UPPER_LEG_RADIUS), 0.1*UPPER_LEG_HEIGHT, 0.0);
  glRotatef(theta[4][0]-theta[0][1], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,lul_node.homogeneous_m);
}

void set_rul(){
  // right upper leg
  glLoadIdentity();
  glTranslatef(TORSO_RADIUS+UPPER_LEG_RADIUS, 0.1*UPPER_LEG_HEIGHT, 0.0);
  glRotatef(theta[4][0]-theta[0][1], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,rul_node.homogeneous_m);
}

void set_lll(){
  // left lower leg
  glLoadIdentity();
  glTranslatef(0.0, -(UPPER_LEG_HEIGHT), 0.0);
  glRotatef(theta[5][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,lll_node.homogeneous_m);
}

void set_rll(){
  // right lower leg
  glLoadIdentity();
  glTranslatef(0.0, -(UPPER_LEG_HEIGHT), 0.0);
  glRotatef(theta[5][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,rll_node.homogeneous_m);
}

void set_wb1i(){
  // weight bar (left, inside)
  glLoadIdentity();
  glTranslatef(0.0, LOWER_ARM_HEIGHT, 0.0);
  glRotatef(theta[6][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,wb1i_node.homogeneous_m);
}

void set_wb1o(){
  // weight bar (left, outside)
  glLoadIdentity();
  glTranslatef(0.0, LOWER_ARM_HEIGHT, 0.0);
  glRotatef(theta[6][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,wb1o_node.homogeneous_m);
}

void set_w1(){
  // weight (left)
  glLoadIdentity();
  glTranslatef(-(WEIGHT_BAR_LENGTH), 0.0, 0.0);
  glRotatef(theta[6][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,w1_node.homogeneous_m);
}

void set_wb2i(){
  // weight bar (right, inside)
  glLoadIdentity();
  glTranslatef(0.0, LOWER_ARM_HEIGHT, 0.0);
  glRotatef(theta[6][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,wb2i_node.homogeneous_m);
}

void set_wb2o(){
  // weight bar (right, outside)
  glLoadIdentity();
  glTranslatef(0.0, LOWER_ARM_HEIGHT, 0.0);
  glRotatef(theta[6][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,wb2o_node.homogeneous_m);
}

void set_w2(){
  // weight (right)
  glLoadIdentity();
  glTranslatef(WEIGHT_BAR_LENGTH, 0.0, 0.0);
  glRotatef(theta[6][0], 1.0, 0.0, 0.0);
  glGetFloatv(GL_MODELVIEW_MATRIX,w2_node.homogeneous_m);
}

void reset(){
  origin[0] = 0.0;
  origin[1] = 0.0;

  theta[0][0] = 0.0;
  theta[0][1] = 0.0;
  theta[1][0] = 0.0;
  theta[1][1] = 0.0;
  theta[2][0] = 0.0;
  theta[2][1] = 0.0;
  theta[3][0] = 0.0;
  theta[3][1] = 0.0;
  theta[4][0] = 0.0;
  theta[4][1] = 0.0;
  theta[5][0] = 0.0;
  theta[5][1] = 0.0;
  theta[6][0] = 0.0;
  theta[6][1] = 0.0;

  glutPostRedisplay();
}

void timerFunc(int val){
  if(playback_ptr<=event_ptr){
    selectionName = event_buffer[playback_ptr].obj_id;
    if (selectionName > 0 && selectionName <= 7){
      theta[selectionName-1][0] = event_buffer[playback_ptr].event_data1;
      theta[selectionName-1][1] = event_buffer[playback_ptr].event_data2;
    } else if (selectionName==8){
      origin[0] = event_buffer[playback_ptr].event_data1;
      origin[1] = event_buffer[playback_ptr].event_data2;
    }
    playback_ptr++;
    display();
    glutTimerFunc(50, timerFunc, 1);
  } else {
    playback_ptr = 0;
  }
}

void myinit()
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glColor3f(1.0, 0.0, 0.0);

/* allocate quadrics with filled drawing style */

  h_q=gluNewQuadric();
  gluQuadricDrawStyle(h_q, GLU_LINE);
  //t_q=gluNewQuadric();
  //gluQuadricDrawStyle(t_q, GLU_LINE);
  abd_q=gluNewQuadric();
  gluQuadricDrawStyle(abd_q, GLU_LINE);
  che_q=gluNewQuadric();
  gluQuadricDrawStyle(che_q, GLU_LINE);
  lua_q=gluNewQuadric();
  gluQuadricDrawStyle(lua_q, GLU_LINE);
  lla_q=gluNewQuadric();
  gluQuadricDrawStyle(lla_q, GLU_LINE);
  rua_q=gluNewQuadric();
  gluQuadricDrawStyle(rua_q, GLU_LINE);
  rla_q=gluNewQuadric();
  gluQuadricDrawStyle(rla_q, GLU_LINE);
  lul_q=gluNewQuadric();
  gluQuadricDrawStyle(lul_q, GLU_LINE);
  lll_q=gluNewQuadric();
  gluQuadricDrawStyle(lll_q, GLU_LINE);
  rul_q=gluNewQuadric();
  gluQuadricDrawStyle(rul_q, GLU_LINE);
  rll_q=gluNewQuadric();
  gluQuadricDrawStyle(rll_q, GLU_LINE);
  wb1i_q=gluNewQuadric();
  gluQuadricDrawStyle(wb1i_q, GLU_LINE);
  wb1o_q=gluNewQuadric();
  gluQuadricDrawStyle(wb1o_q, GLU_LINE);
  w1_q=gluNewQuadric();
  gluQuadricDrawStyle(w1_q, GLU_LINE);
  wb2i_q=gluNewQuadric();
  gluQuadricDrawStyle(wb2i_q, GLU_LINE);
  wb2o_q=gluNewQuadric();
  gluQuadricDrawStyle(wb2o_q, GLU_LINE);
  w2_q=gluNewQuadric();
  gluQuadricDrawStyle(w2_q, GLU_LINE);

/* Set up tree */
  torso_node.id_number = 1; 
  torso_node.setting_f = set_t;
  torso_node.setting_f();
  torso_node.drawing_f = torso_f;
  torso_node.sibling = TREE_NULL;
  torso_node.child =  &head_node;

  head_node.id_number = 2;
  head_node.setting_f = set_h;
  head_node.setting_f();
  head_node.drawing_f = head_f;
  head_node.sibling = &lua_node;
  head_node.child = TREE_NULL;

  lua_node.id_number = 3;
  lua_node.setting_f = set_lua;
  lua_node.setting_f();
  lua_node.drawing_f = left_upper_arm_f;
  lua_node.sibling =  &rua_node;
  lua_node.child = &lla_node;

  rua_node.id_number = 3;
  rua_node.setting_f = set_rua;
  rua_node.setting_f();
  rua_node.drawing_f = right_upper_arm_f;
  rua_node.sibling =  &lul_node;
  rua_node.child = &rla_node;

  lul_node.id_number = 5;
  lul_node.setting_f = set_lul;
  lul_node.setting_f();
  lul_node.drawing_f = left_upper_leg_f;
  lul_node.sibling =  &rul_node;
  lul_node.child = &lll_node;

  rul_node.id_number = 5;
  rul_node.setting_f = set_rul;
  rul_node.setting_f();
  rul_node.drawing_f = right_upper_leg_f;
  rul_node.sibling =  TREE_NULL;
  rul_node.child = &rll_node;

  lla_node.id_number = 4;
  lla_node.setting_f = set_lla;
  lla_node.setting_f();
  lla_node.drawing_f = &left_lower_arm_f;
  lla_node.sibling =  TREE_NULL;
  lla_node.child = &wb1i_node;

  rla_node.id_number = 4;
  rla_node.setting_f = set_rla;
  rla_node.setting_f();
  rla_node.drawing_f = &right_lower_arm_f;
  rla_node.sibling =  TREE_NULL;
  rla_node.child = &wb2i_node;

  lll_node.id_number = 6;
  lll_node.setting_f = set_lll;
  lll_node.setting_f();
  lll_node.drawing_f = &left_lower_leg_f;
  lll_node.sibling =  TREE_NULL;
  lll_node.child = TREE_NULL;

  rll_node.id_number = 6;
  rll_node.setting_f = set_rll;
  rll_node.setting_f();
  rll_node.drawing_f = &right_lower_leg_f;
  rll_node.sibling =  TREE_NULL;
  rll_node.child = TREE_NULL;

  wb1i_node.id_number = 7;
  wb1i_node.setting_f = set_wb1i;
  wb1i_node.setting_f();
  wb1i_node.drawing_f = &weight_bar_1_inside_f;
  wb1i_node.sibling =  &wb1o_node;
  wb1i_node.child = TREE_NULL;

  wb1o_node.id_number = 7;
  wb1o_node.setting_f = set_wb1o;
  wb1o_node.setting_f();
  wb1o_node.drawing_f = &weight_bar_1_outside_f;
  wb1o_node.sibling =  TREE_NULL;
  wb1o_node.child = &w1_node;

  w1_node.id_number = 7;
  w1_node.setting_f = set_w1;
  w1_node.setting_f();
  w1_node.drawing_f = &weight_1_f;
  w1_node.sibling =  TREE_NULL;
  w1_node.child = TREE_NULL;

  wb2i_node.id_number = 7;
  wb2i_node.setting_f = set_wb2i;
  wb2i_node.setting_f();
  wb2i_node.drawing_f = &weight_bar_2_inside_f;
  wb2i_node.sibling =  &wb2o_node;
  wb2i_node.child = TREE_NULL;

  wb2o_node.id_number = 7;
  wb2o_node.setting_f = set_wb2o;
  wb2o_node.setting_f();
  wb2o_node.drawing_f = &weight_bar_2_outside_f;
  wb2o_node.sibling =  TREE_NULL;
  wb2o_node.child = &w2_node;

  w2_node.id_number = 7;
  w2_node.setting_f = set_w2;
  w2_node.setting_f();
  w2_node.drawing_f = &weight_2_f;
  w2_node.sibling =  TREE_NULL;
  w2_node.child = TREE_NULL;
}

void moveObject(int x, int y){
  if(currentButton == GLUT_LEFT_BUTTON){
    theta[selectionName-1][0] += 5.0;
    printf("angle is now %f.\n", theta[selectionName-1][0]);
    if(theta[selectionName-1][0]>360.0) theta[selectionName-1][0] -= 360.0;
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = theta[selectionName-1][0];
      event_buffer[event_ptr].event_data2 = theta[selectionName-1][1];
      event_ptr++;
    }
    display();
  } else if(currentButton == GLUT_RIGHT_BUTTON) {
    theta[selectionName-1][0] -= 5.0;
    printf("angle is now %f.\n", theta[selectionName-1][0]);
    if(theta[selectionName-1][0] <0.0) theta[selectionName-1][0] += 360.0;
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = theta[selectionName-1][0];
      event_buffer[event_ptr].event_data2 = theta[selectionName-1][1];
      event_ptr++;
    }
    display();
  }else if(currentButton == GLUT_MIDDLE_BUTTON && selectionName > 0 && selectionName < 8){
    if(x > setPoint[0]){
      origin[0] = origin[0] + 0.1;
      setPoint[0] = setPoint[0] + 0.1;
    } else {
      origin[0] = origin[0] - 0.1;
      setPoint[0] = setPoint[0] - 0.1;
    } 
    if(y > setPoint[1]) {
      origin[1] = origin[1] - 0.1;
      setPoint[1] = setPoint[1] - 0.1;
    } else {
      origin[1] = origin[1] + 0.1;
      setPoint[1] = setPoint[1] + 0.1;
    }
    if(recordMode == 1){
      event_buffer[event_ptr].obj_id = selectionName;
      event_buffer[event_ptr].event_data1 = origin[0];
      event_buffer[event_ptr].event_data2 = origin[1];
      event_ptr++;
    }
    //printf("origin is at %f and %f.\n", origin[0], origin[1]);
    //printf("cursor is at %d and %d.\n", x, y);
    display();
  }
  
  glFlush();
}

void keyboard(unsigned char key, int x, int y)
{
  int fileSuccessfullyRead1 = 0;
  int fileSuccessfullyRead2 = 0;
  int fileSuccessfullyRead3 = 0;
  switch(key) {
  case 'b':
    recordMode = 1;
    playbackMode = 0;
    printf("Recording... Press 'e' to end\n");
    event_ptr = 0;
    break;
  case 'e':
    if (recordMode==1){
      recordMode = 0;
      printf("Recording stopped. Press 's' to save to playback file.\n");
    }
    break;
  case 'l':
    recordMode = 0;
    playbackMode = 0;
    event_ptr = 0;
    playback_ptr = 0;
    reset();
    printf("Loading file %s\n", fileName);
    jFile = fopen(fileName, "r");
    if (jFile == NULL){
      printf("Warning: Could not open %s\n", fileName);
      playbackMode = 0;
    } else {
      do {
	fileSuccessfullyRead1 = fscanf(jFile, "%d", &event_buffer[event_ptr].obj_id);
	fileSuccessfullyRead2 = fscanf(jFile, "%f", &event_buffer[event_ptr].event_data1);
	fileSuccessfullyRead3 = fscanf(jFile, "%f", &event_buffer[event_ptr].event_data2);
	event_ptr++;
      } while (fileSuccessfullyRead1 != EOF && fileSuccessfullyRead2 != EOF && fileSuccessfullyRead3 != EOF);
      fclose(jFile);
      playbackMode = 1;
    }
    break;
  case 'r':
    recordMode = 0;
    playbackMode = 0;
    event_ptr = 0;
    playback_ptr = 0;
    reset();
    break;
  case 'p':
    if (playbackMode == 1){
      reset();
      glutTimerFunc(4, timerFunc, 1);
    }
    break;
  case 's':
    recordMode = 0;
    playbackMode = 0;
    jFile = fopen(fileName, "w");
    if (jFile == NULL){
      printf("Warning: Could not open %s\n", fileName);
    } else {
      for(int j=0; j<event_ptr; j++){
	fprintf(jFile, "%d ", event_buffer[j].obj_id);
	fprintf(jFile, "%f ", event_buffer[j].event_data1);
	fprintf(jFile, "%f ", event_buffer[j].event_data2);
      }
      fclose(jFile);
      printf("\nEvents saved in %s \n", fileName);
    }
    playback_ptr=0;
    break;
  }
}

void main(int argc, char **argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  glutCreateWindow("robot");
  myinit();
  glutReshapeFunc(myReshape);
  glutDisplayFunc(display);
  glutMouseFunc(mouse);
  glutMotionFunc(moveObject);
  glutKeyboardFunc(keyboard);

  glutMainLoop();
}
