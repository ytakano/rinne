#include "rinne.hpp"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include <GL/glut.h>
#endif

#include <iostream>
#include <fstream>
#include <string>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::edge_iterator edge_iter;

#define DISTANCE2(D, A, B) do {                 \
        double x2, y2, z2;                      \
        x2 = A.x - B.x;                         \
        y2 = A.y - B.y;                         \
        z2 = A.z - B.z;                         \
        x2 *= x2;                               \
        y2 *= y2;                               \
        z2 *= z2;                               \
        D = x2 + y2 + z2;                       \
    } while (0)

#define CROSS_PRODUCT(D, A, B) do {             \
        D.x = A.y * B.z - A.z * B.y;            \
        D.y = A.z * B.x - A.x * B.z;            \
        D.z = A.x * B.y - A.y * B.x;            \
    } while (0)

#define TO_SPHERICAL(D, A, R) do {                      \
        double d = sqrt(A.x * A.x + A.y * A.y);         \
        if (d > 0.0) {                                  \
            D.phi = acos(A.x / d);                      \
        } else {                                        \
            D.phi = 0.0;                                \
        }                                               \
        D.theta = acos(A.z / R);                        \
    } while (0)

#define TO_RECTANGULAR(D, A, R) do {           \
        double sin_theta = sin(A.theta);       \
        D.x = R * sin_theta * cos(A.phi);      \
        D.y = R * sin_theta * sin(A.phi);      \
        D.z = R * cos(A.theta);                \
    } while (0)

#define GET_UV(U, V, A) do {                   \
        double cos_theta = cos(A.theta);       \
        double cos_phi   = cos(A.phi);         \
        double sin_phi   = sin(A.phi);         \
        U.x = - cos_theta * cos_phi;           \
        U.y = - cos_theta * sin_phi;           \
        U.z = sin(A.theta);                    \
        V.x = sin_phi;                         \
        V.y = - cos_phi;                       \
        V.z = 0.0;                             \
    } while (0)

#define QUATERNION_MUL(D, A, B) do {                            \
        D.w = A.w * B.w - A.i * B.i - A.j * B.j - A.k * B.k;    \
        D.i = A.i * B.w + A.w * B.i - A.k * B.j + A.j * B.k;    \
        D.j = A.j * B.w + A.k * B.i + A.w * B.j - A.i * B.k;    \
        D.k = A.k * B.w - A.j * B.i + A.i * B.j + A.w * B.k;    \
    } while (0)

#define ROTATE(A, V, RAD) do {                  \
        rn_quaternion p, q, r;                  \
        double r2 = RAD * -0.5;                 \
        double sin_rad2 = sin(r2);              \
        double cos_rad2 = cos(r2);              \
                                                \
        p.w = 0.0;                              \
        p.i = A.x;                              \
        p.j = A.y;                              \
        p.k = A.z;                              \
                                                \
        q.w = cos_rad2;                         \
        q.i = v.x * sin_rad2;                   \
        q.j = v.y * sin_rad2;                   \
        q.k = v.z * sin_rad2;                   \
                                                \
        r.w = cos_rad2;                         \
        r.i = - q.i;                            \
        r.j = - q.j;                            \
        r.k = - q.k;                            \
                                                \
        rn_quaternion tmp, result;              \
                                                \
        QUATERNION_MUL(tmp, r, p);              \
        QUATERNION_MUL(result, tmp, q);         \
                                                \
        A.x = result.i;                         \
        A.y = result.j;                         \
        A.z = result.k;                         \
    } while (0)

#define NORMALIZE(V) do {                                       \
        double d = sqrt(V.x * V.x + V.y * V.y + V.z * V.z);     \
        if (d > 0.0) {                                          \
            V.x /= d;                                           \
            v.y /= d;                                           \
            v.z /= d;                                           \
        } else {                                                \
            v.x = v.y = v.z = 0.0;                              \
        }                                                       \
    } while (0)

rn_node *node_pool;
rn_edge *edge_pool;

rinne rinne_inst;

void
cpu_rotate(rn_vec &a, const rn_vec &v, double rad)
{
    ROTATE(a, v, rad);
}

void
rinne::display()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush();

    glPushMatrix();

    glRotated(360 * m_rotate_z, 0.0, 0.0, 1.0);
    glRotated(360 * m_rotate_x, 1.0, 0.0, 0.0);

    glColor3d(0.4, 0.4, 0.4);
    glutWireSphere(1.0, 16, 16);

    draw_node();
    draw_tau();

    glPopMatrix();

    glutSwapBuffers();
}

void
display()
{
    rinne_inst.display();
}

void
on_keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case 'q':
    case 'Q':
    case '\033': // ESC
        exit(0);
    default:
        break;
    }
}

void
on_resize(int w, int h)
{
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)w / (double)h, 0.1, 8);
    gluLookAt(0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
}

void
on_mouse(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN) {
        rinne_inst.on_mouse_down(button, x, y);
    } else if (state == GLUT_UP) {
        rinne_inst.on_mouse_up(button, x, y);
    }
}

void
on_mouse_move(int x, int y)
{
    rinne_inst.on_mouse_move(x, y);
}

void
init_glut(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(display);
    glutKeyboardFunc(on_keyboard);
    glutReshapeFunc(on_resize);
    glutMouseFunc(on_mouse);
    glutMotionFunc(on_mouse_move);
    glutFullScreen();
    glutMainLoop();
}

void
rinne::on_mouse_down(int button, int x, int y)
{
    m_is_mouse_down = true;
    m_mouse_x = x;
    m_mouse_y = y;
}

void
rinne::on_mouse_up(int button, int x, int y)
{
    m_is_mouse_down = false;
}

void
rinne::on_mouse_move(int x, int y)
{
    if (m_is_mouse_down) {
        int dx = x - m_mouse_x;
        int dy = m_mouse_y - y;

        m_rotate_z += dx * 0.001;
        m_rotate_x += dy * 0.001;

        double tmp;
        m_rotate_z = modf(m_rotate_z, &tmp);
        if (m_rotate_z < 0.0) {
            m_rotate_z += 1.0;
        }

        m_rotate_x = modf(m_rotate_x, &tmp);
        if (m_rotate_x < 0.0) {
            m_rotate_x += 1.0;
        }

        m_mouse_x = x;
        m_mouse_y = y;

        glutPostRedisplay();
    }
}

void
rinne::draw_tau()
{
    if (m_num_node == 0)
        return;

    rn_vec a;
    TO_RECTANGULAR(a, m_node[0].pos, 1.0);

    double cos_theta_a = cos(m_node[0].pos.theta);
    double sin_theta_a = sin(m_node[0].pos.theta);

    rn_vec au, av;

    GET_UV(au, av, m_node[0].pos);

    for (rn_node *p = m_node + 1; p < &m_node[m_num_node]; p++) {
        rn_vec b;
        double  t, x, y, z;

        TO_RECTANGULAR(b, p->pos, 1.0);

        t = 1.0 - a.x * b.x - a.y * b.y - a.z * b.z;

        x = b.x + a.x * t;
        y = b.y + a.y * t;
        z = b.z + a.z * t;

        glPushMatrix();
        glColor3d(1.0, 1.0, 0.0);
        glTranslated(x, y, z);
        glutWireSphere(0.1, 16, 16);
        glPopMatrix();

        glBegin(GL_LINES);
        glVertex3d(b.x, b.y, b.z);
        glVertex3d(x, y, z);
        glEnd();

        double psi = acos(cos_theta_a * cos(p->pos.theta) +
                          sin_theta_a * sin(p->pos.theta) *
                          cos(m_node[0].pos.phi - p->pos.phi));
        rn_vec ba;
        ba.x = x - a.x;
        ba.y = y - a.y;
        ba.z = z - a.z;

        double dist;
        rn_vec origin = {0.0, 0.0, 0.0};
        DISTANCE2(dist, ba, origin);

        ba.x /= sqrt(dist);
        ba.y /= sqrt(dist);
        ba.z /= sqrt(dist);

        ba.x *= psi;
        ba.y *= psi;
        ba.z *= psi;

        glPushMatrix();
        glColor3d(1.0, 0.0, 1.0);
        glTranslated(a.x, a.y, a.z);

        glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(ba.x, ba.y, ba.z);
        glEnd();

        glPopMatrix();

        glPushMatrix();
        glColor3d(1.0, 0.0, 1.0);
        glTranslated(ba.x + a.x, ba.y + a.y, ba.z + a.z);
        glutWireSphere(0.1, 16, 16);
        glPopMatrix();

        rn_vec cross;

        CROSS_PRODUCT(cross, a, ba);
        DISTANCE2(dist, cross, origin);

        cross.x /= sqrt(dist);
        cross.y /= sqrt(dist);
        cross.z /= sqrt(dist);

        rn_vec ra = a;
        double rad = psi * 0.5;
        cpu_rotate(ra, cross, rad);

        glPushMatrix();
        glColor3d(0.0, 1.0, 1.0);
        glTranslated(ra.x, ra.y, ra.z);
        glutWireSphere(0.1, 16, 16);
        glPopMatrix();


        rn_uv buv;
        double bb;

        std::cout << "(x, y, z) = ("
                  << x << ", " << y << ", " << z << ")"
                  << std::endl;

        buv.u = au.x * x + au.y * y + au.z * z;
        buv.v = av.x * x + av.y * y + av.z * z;
        bb    = a.x * x + a.y * y + a.z * z;

        std::cout << "u = " << buv.u << std::endl;
        std::cout << "v = " << buv.v << std::endl;
        std::cout << "a = " << bb << std::endl;

        //buv.u *= 0.5;
        //buv.v *= 0.5;

        rn_vec b2;

        b2.x = au.x * buv.u + av.x * buv.v + a.x * bb;
        b2.y = au.y * buv.u + av.y * buv.v + a.y * bb;
        b2.z = au.z * buv.u + av.z * buv.v + a.z * bb;

        std::cout << "(x', y', z') = ("
                  << b2.x << ", " << b2.y << ", " << b2.z << ")\n"
                  << std::endl;
/*
        glPushMatrix();
        glColor3d(1.0, 0.0, 1.0);
        glTranslated(b2.x, b2.y, b2.z);
        glutWireSphere(0.1, 16, 16);
        glPopMatrix();

        std::cout << "(x, y, z) = (" << x << ", " << y << ", " << z << ")"
                  << std::endl;

        rn_uv buv;
        double bb;

        buv.u = au.x * x + av.x * y + a.x * z;
        buv.v = au.y * x + av.y * y + a.y * z;
        bb    = au.z * x + av.z * y + a.z * z;

        std::cout << "u = " << buv.u << std::endl;
        std::cout << "v = " << buv.v << std::endl;
        std::cout << "a = " << bb << std::endl;

        rn_vec b2;

        b2.x = au.x * buv.u + au.y * buv.v + au.z;
        b2.y = av.x * buv.u + av.y * buv.v + av.z;
        b2.z = a.x  * buv.u + a.y  * buv.v + a.z;

        std::cout << "(x', y', z') = ("
                  << b2.x << ", " << b2.y << ", " << b2.z << ")\n"
                  << std::endl;
*/
    }
}

void
rinne::draw_node()
{
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        rn_vec a, u, v;
        TO_RECTANGULAR(a, p->pos, 1.0);
        GET_UV(u, v, p->pos);

        glPushMatrix();

        glTranslated(a.x, a.y, a.z);
        glColor3d(1.0, 0.0, 0.0);
        glutWireSphere(0.1, 16, 16);

        glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(a.x * 0.5, a.y * 0.5 , a.z * 0.5);
        glEnd();

        glColor3d(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(u.x * 0.5, u.y * 0.5 , u.z * 0.5);
        glEnd();

        glColor3d(0.0, 0.0, 1.0);
        glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(v.x * 0.5, v.y * 0.5, v.z * 0.5);
        glEnd();

        glPopMatrix();
    }
}

void
rinne::cpu_spring_v0()
{

}

void
rinne::init_pos()
{
    srand(time(NULL));
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        p->pos.theta = M_PI * ((double)rand() / RAND_MAX);
        p->pos.phi   = 2 * M_PI * ((double)rand() / RAND_MAX);
    }
}

void
rinne::read_dot(char *path)
{
    std::ifstream file(path);

    Graph g;
    boost::dynamic_properties dp(boost::ignore_other_properties);
    boost::read_graphviz(file, g, dp);

    m_num_node = num_vertices(g);
    m_num_edge = num_edges(g);

    m_node = new rn_node[m_num_node];
    m_edge = new rn_edge[m_num_edge];

/*
    gpuErrchk(cudaMallocManaged((void**)&m_node,
                                sizeof(*m_node) * m_num_node,
                                cudaMemAttachGlobal));
    gpuErrchk(cudaMallocManaged((void**)&m_edge,
                                sizeof(*m_edge) * m_num_edge,
                                cudaMemAttachGlobal));
*/

    memset(m_node, 0, sizeof(*m_node) * m_num_node);
    memset(m_edge, 0, sizeof(*m_edge) * m_num_edge);

    std::pair<edge_iter, edge_iter> ep;
    rn_edge *p_edge = m_edge;
    for (ep = edges(g); ep.first != ep.second; ++ep.first) {
        int s = source(*ep.first, g);
        int t = target(*ep.first, g);

        p_edge->src = &m_node[s];
        p_edge->dst = &m_node[t];

        p_edge->next = p_edge->src->edge;
        p_edge->src->edge = p_edge;

        p_edge++;
    }

    init_pos();
}

int
main(int argc, char *argv[])
{
    rn_vec a, v;

    a.x = 1, a.y = 0, a.z = 0;
    v.x = 0, v.y = 0, v.z = 1;

    cpu_rotate(a, v, M_PI_4 * 7);

    std::cout << a.x << "," << a.y << "," << a.z << std::endl;

    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " graph.dot" << std::endl;
        return 1;
    }

    std::cout << "loading " << argv[1] << " ..." << std::endl;
    rinne_inst.read_dot(argv[1]);

    init_glut(argc, argv);

    return 0;
}
