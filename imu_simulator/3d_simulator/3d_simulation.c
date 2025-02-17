#include <GL/glut.h>
#include <stdio.h>

float xAngle = 0.0f;
float yAngle = 0.0f;
float zAngle = 0.0f;

void renderText(float x, float y, float z, const char* text) {
    glRasterPos3f(x, y, z);
    for (const char* c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0.0f, 0.0f, -5.0f);

    // Draw auxiliary lines (Gray, static)
    glColor3f(0.5f, 0.5f, 0.5f);
    glBegin(GL_LINES);
    for (float i = -1.0f; i <= 1.0f; i += 0.5f) {
        // Parallel to X axis
        glVertex3f(i, -1.0f, 0.0f);
        glVertex3f(i, 1.0f, 0.0f);
        // Parallel to Y axis
        glVertex3f(-1.0f, i, 0.0f);
        glVertex3f(1.0f, i, 0.0f);
    }
    glEnd();

    // Label axes (static)
    glColor3f(1.0f, 1.0f, 1.0f);
    renderText(1.1f, 0.0f, 0.0f, "X");
    renderText(0.0f, 1.1f, 0.0f, "Y");
    renderText(0.0f, 0.0f, 1.1f, "Z");

    // Apply rotations to XYZ axes
    glPushMatrix();
    glRotatef(xAngle, 1.0f, 0.0f, 0.0f);
    glRotatef(yAngle, 0.0f, 1.0f, 0.0f);
    glRotatef(zAngle, 0.0f, 0.0f, 1.0f);

    // Draw X axis (Red)
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glEnd();

    // Draw Y axis (Green)
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glEnd();

    // Draw Z axis (Blue)
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    glPopMatrix();

    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / (float)h, 1.0f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 'x':
            xAngle += 5.0f;
            break;
        case 'X':
            xAngle -= 5.0f;
            break;
        case 'y':
            yAngle += 5.0f;
            break;
        case 'Y':
            yAngle -= 5.0f;
            break;
        case 'z':
            zAngle += 5.0f;
            break;
        case 'Z':
            zAngle -= 5.0f;
            break;
        case 27: // ESC key
            exit(0);
            break;
    }
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Axis Rotation");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}

