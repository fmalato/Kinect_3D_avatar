#include <GLFW/glfw3.h>

void drawGrid() {

    glLineWidth(3);
    glColor3f(1,1,1);
    glBegin(GL_LINE_LOOP);
    glVertex3f( 0,-0.001, 0);
    glVertex3f( 0,-0.001,10);
    glVertex3f(10,-0.001,10);
    glVertex3f(10,-0.001, 0);
    glEnd();

    glBegin(GL_LINES);
    for(int i=0;i<=10;i++) {
        if (i==0) { glColor3f(.6,.3,.3); } else { glColor3f(.25,.25,.25); };
        glVertex3f(i,0,0);
        glVertex3f(i,0,10);
        if (i==0) { glColor3f(.3,.3,.6); } else { glColor3f(.25,.25,.25); };
        glVertex3f(0,0,i);
        glVertex3f(10,0,i);
    };
    glEnd();

}