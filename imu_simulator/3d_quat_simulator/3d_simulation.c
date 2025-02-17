#include <GL/glut.h>
#include <stdio.h>
#include <math.h>

// 定義四元數結構
typedef struct {
    float w, x, y, z;
} Quaternion;

// 當前四元數（初始為單位四元數）
Quaternion rotation = {1.0f, 0.0f, 0.0f, 0.0f};

// 四元數乘法（q1 * q2）
Quaternion quat_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// 正規化四元數（防止數值誤差）
Quaternion quat_normalize(Quaternion q) {
    float mag = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (mag > 0.0f) {
        q.w /= mag;
        q.x /= mag;
        q.y /= mag;
        q.z /= mag;
    }
    return q;
}

// 旋轉角度轉換為四元數（角度以度為單位）
Quaternion quat_from_axis_angle(float angle, float x, float y, float z) {
    Quaternion q;
    float rad = angle * M_PI / 180.0f;
    float s = sin(rad / 2.0f);
    q.w = cos(rad / 2.0f);
    q.x = x * s;
    q.y = y * s;
    q.z = z * s;
    return q;
}

// 將四元數轉換為旋轉矩陣並應用
void apply_rotation() {
    float mat[16];
    mat[0]  = 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z);
    mat[1]  = 2 * (rotation.x * rotation.y - rotation.w * rotation.z);
    mat[2]  = 2 * (rotation.x * rotation.z + rotation.w * rotation.y);
    mat[3]  = 0;
    
    mat[4]  = 2 * (rotation.x * rotation.y + rotation.w * rotation.z);
    mat[5]  = 1 - 2 * (rotation.x * rotation.x + rotation.z * rotation.z);
    mat[6]  = 2 * (rotation.y * rotation.z - rotation.w * rotation.x);
    mat[7]  = 0;
    
    mat[8]  = 2 * (rotation.x * rotation.z - rotation.w * rotation.y);
    mat[9]  = 2 * (rotation.y * rotation.z + rotation.w * rotation.x);
    mat[10] = 1 - 2 * (rotation.x * rotation.x + rotation.y * rotation.y);
    mat[11] = 0;
    
    mat[12] = 0;
    mat[13] = 0;
    mat[14] = -5;
    mat[15] = 1;
    
    glMultMatrixf(mat);
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / (float)h, 1.0f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    apply_rotation(); // 應用四元數旋轉
    
    // 畫三軸
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y) {
    Quaternion q;
    switch (key) {
        case 'x': q = quat_from_axis_angle(5.0f, 1.0f, 0.0f, 0.0f); break;
        case 'X': q = quat_from_axis_angle(-5.0f, 1.0f, 0.0f, 0.0f); break;
        case 'y': q = quat_from_axis_angle(5.0f, 0.0f, 1.0f, 0.0f); break;
        case 'Y': q = quat_from_axis_angle(-5.0f, 0.0f, 1.0f, 0.0f); break;
        case 'z': q = quat_from_axis_angle(5.0f, 0.0f, 0.0f, 1.0f); break;
        case 'Z': q = quat_from_axis_angle(-5.0f, 0.0f, 0.0f, 1.0f); break;
        case 27: exit(0);
    }
    rotation = quat_multiply(q, rotation); // 修正為 q * rotation
    rotation = quat_normalize(rotation); // 正規化防止數值誤差
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Quaternion Rotation");
    
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    
    glutMainLoop();
    return 0;
}

