#include "rinne.hpp"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

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
#include <boost/thread.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::edge_iterator edge_iter;

#define TIMERSEC 16
#define SQRTPI 0.5641898
#define CAMERA_Y -3.0
#define NODE_R_MAX 0.05
#define NODE_R_MIN 0.003
#define NODE_R_DIFF (NODE_R_MAX - NODE_R_MIN)

#define DISTANCE2(D, A) do {                    \
        double x2, y2, z2;                      \
        x2 = (A).x * (A).x;                     \
        y2 = (A).y * (A).y;                     \
        z2 = (A).z * (A).z;                     \
        (D) = x2 + y2 + z2;                     \
    } while (0)

#define CROSS_PRODUCT(D, A, B) do {             \
        (D).x = (A).y * (B).z - (A).z * (B).y;  \
        (D).y = (A).z * (B).x - (A).x * (B).z;  \
        (D).z = (A).x * (B).y - (A).y * (B).x;  \
    } while (0)

#define TO_SPHERICAL(D, A) do {                         \
        double tmp = (A).x * (A).x + (A).y * (A).y;     \
        double r   = sqrt(tmp + (A).z * (A).z);         \
        double rxy = sqrt(tmp);                         \
        if (r > 0.0) {                                  \
            (D).theta = acos((A).z / r);                \
            (D).phi = acos((A).x / rxy);                \
            if ((A).y < 0.0) {                          \
                (D).phi = 2 * M_PI - (D).phi;           \
            }                                           \
        } else {                                        \
            (D).theta = 0.0;                            \
            (D).phi   = 0.0;                            \
        }                                               \
    } while (0)

#define TO_RECTANGULAR(D, A, R) do {            \
        double sin_theta = sin((A).theta);      \
        (D).x = (R) * sin_theta * cos((A).phi); \
        (D).y = (R) * sin_theta * sin((A).phi); \
        (D).z = (R) * cos((A).theta);           \
    } while (0)

#define GET_UV(U, V, A) do {                   \
        double cos_theta = cos((A).theta);     \
        double cos_phi   = cos((A).phi);       \
        double sin_phi   = sin((A).phi);       \
        (U).x = - cos_theta * cos_phi;         \
        (U).y = - cos_theta * sin_phi;         \
        (U).z = sin(A.theta);                  \
        (V).x = sin_phi;                       \
        (V).y = - cos_phi;                     \
        (V).z = 0.0;                           \
    } while (0)

#define QUATERNION_MUL(D, A, B) do {                                    \
        (D).w = (A).w * (B).w - (A).i * (B).i - (A).j * (B).j - (A).k * (B).k; \
        (D).i = (A).i * (B).w + (A).w * (B).i - (A).k * (B).j + (A).j * (B).k; \
        (D).j = (A).j * (B).w + (A).k * (B).i + (A).w * (B).j - (A).i * (B).k; \
        (D).k = (A).k * (B).w - (A).j * (B).i + (A).i * (B).j + (A).w * (B).k; \
    } while (0)

#define ROTATE(A, V, RAD) do {                  \
        rn_quaternion p, q, r;                  \
        double r2 = (RAD) * -0.5;               \
        double sin_rad2 = sin(r2);              \
        double cos_rad2 = cos(r2);              \
                                                \
        p.w = 0.0;                              \
        p.i = (A).x;                            \
        p.j = (A).y;                            \
        p.k = (A).z;                            \
                                                \
        q.w = cos_rad2;                         \
        q.i = (V).x * sin_rad2;                 \
        q.j = (V).y * sin_rad2;                 \
        q.k = (V).z * sin_rad2;                 \
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
        (A).x = result.i;                       \
        (A).y = result.j;                       \
        (A).z = result.k;                       \
    } while (0)

#define NORMALIZE(V) do {                                               \
        double d = sqrt((V).x * (V).x + (V).y * (V).y + (V).z * (V).z); \
        if (d > 0.0001) {                                               \
            d = 1.0 / d;                                                \
            (V).x *= d;                                                 \
            (V).y *= d;                                                 \
            (V).z *= d;                                                 \
        } else {                                                        \
            (V).x = (V).y = (V).z = 0.0;                                \
        }                                                               \
    } while (0)

rn_node *node_pool;
rn_edge *edge_pool;

rinne rinne_inst;
boost::thread thread;

void
run()
{
    int i = 0;
    for (;;) {
        rinne_inst.force_directed();
        //std::cout << ++i << std::endl;
        if (i == 25) {
            rinne_inst.reduce_step();
        } else if (i == 50) {
            rinne_inst.reduce_step();
        } else if (i == 100) {
            rinne_inst.reduce_step();
        }
        //usleep(100000);
    }
}

void
rinne::display()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h, 0.1, 8);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();

    glRotated(360 * m_rotate_z, 0.0, 0.0, 1.0);
    glRotated(360 * m_rotate_x, 1.0, 0.0, 0.0);

    glColor3d(0.4, 0.4, 0.4);
    glutWireSphere(1.0, 16, 16);

    draw_node();
    //draw_tau();

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
    rinne_inst.on_resize(w, h);
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
glut_timer(int val)
{
    glutPostRedisplay();
    glutTimerFunc(TIMERSEC, glut_timer, 0);
}

void
init_glut(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(1024, 768);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(display);
    glutKeyboardFunc(on_keyboard);
    glutReshapeFunc(on_resize);
    glutMouseFunc(on_mouse);
    glutMotionFunc(on_mouse_move);
    glutTimerFunc(TIMERSEC, glut_timer, 0);

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    //glEnable(GL_DEPTH_TEST);
    glutFullScreen();
    glutMainLoop();
}

void
rinne::on_resize(int w, int h)
{
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)w / (double)h, 0.1, 8);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    m_window_w = w;
    m_window_h = h;
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
        int dy = y - m_mouse_y;

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
        DISTANCE2(dist, ba);

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


        rn_uv buv;
        double bb;

        buv.u = au.x * x + au.y * y + au.z * z;
        buv.v = av.x * x + av.y * y + av.z * z;
        bb    = a.x * x + a.y * y + a.z * z;

        //buv.u *= 0.5;
        //buv.v *= 0.5;

        rn_vec b2;

        b2.x = au.x * buv.u + av.x * buv.v + a.x * bb;
        b2.y = au.y * buv.u + av.y * buv.v + a.y * bb;
        b2.z = au.z * buv.u + av.z * buv.v + a.z * bb;

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
rinne::draw_edge()
{
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        for (rn_edge *p_edge = p->edge; p_edge != NULL; p_edge = p_edge->next) {
            rn_vec ev;
            rn_pos delta;

            delta.theta = p->pos.theta - p_edge->dst->pos.theta;
            delta.phi   = p->pos.phi - p_edge->dst->pos.phi;

            if (delta.phi > M_PI) {
                delta.phi = -(2 * M_PI - delta.phi);
            } else if (delta.phi < -M_PI) {
                delta.phi = 2 * M_PI + delta.phi;
            }
            
            TO_RECTANGULAR(ev, p_edge->dst->pos, 1.0);
            glBegin(GL_LINE_STRIP);
            glVertex3d(ev.x, ev.y, ev.z);

            rn_pos pos_arc = p_edge->dst->pos;
            int num_lines = 6;

            delta.theta /= num_lines;
            delta.phi   /= num_lines;

            for (int i = 1; i < num_lines; i++) {
                pos_arc.theta += delta.theta;
                pos_arc.phi   += delta.phi;

                TO_RECTANGULAR(ev, pos_arc, 1.0);
                glVertex3d(ev.x, ev.y , ev.z);
            }

            TO_RECTANGULAR(ev, p->pos, 1.0);
            glVertex3d(ev.x, ev.y , ev.z);

            glEnd();
        }
    }
}

void
rinne::draw_node()
{
    double r_denom = 1.0 / m_max_in_degree;
    double max_alpha = 1.0;
    double min_alpha = 0.0;
    double max_g;
    double min_g;
    double max_b;
    double min_b;
    double cycle = 10.0;
    double b, g, alpha;

    // draw far side nodes
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h, -CAMERA_Y, 8);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glColor3d(0.0, 0.4, 0.1);
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        rn_vec a, u, v;
        double r = p->num_bp_edge * r_denom;

        r = r * NODE_R_DIFF + NODE_R_MIN;

        TO_RECTANGULAR(a, p->pos, 1.0);

        GET_UV(u, v, p->pos);

        glPushMatrix();

        glTranslated(a.x, a.y, a.z);
        glutSolidSphere(r, 8, 8);

        glPopMatrix();
    }

    max_alpha = 1.0;
    min_alpha = 0.15;
    max_g = 0.7;
    min_g = 0.06;
    max_b = 0.4;
    min_b = 0.0;

    get_color(g, b, alpha, min_g, max_g, min_b, max_b, min_alpha, max_alpha,
              cycle);

    // draw edges
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h,
                   0.1, -CAMERA_Y);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glColor4d(0.0, g, b, alpha);
    draw_edge();


    // draw near side nodes
    max_g = 0.9;
    min_g = 0.5;
    max_b = 0.6;
    min_b = 0.2;

    get_color(g, b, alpha, min_g, max_g, min_b, max_b, min_alpha, max_alpha,
              cycle);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h,
                   0.1, -CAMERA_Y);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glColor3d(0.0, g, b);
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        rn_vec a, u, v;
        double r = p->num_bp_edge * r_denom;

        r = r * NODE_R_DIFF + NODE_R_MIN;

        TO_RECTANGULAR(a, p->pos, 1.0);

        GET_UV(u, v, p->pos);

        glPushMatrix();

        glTranslated(a.x, a.y, a.z);
        glutSolidSphere(r, 8, 8);

        glPopMatrix();
    }
}

void
rinne::get_spring_vec(rn_vec &uv, double psi)
{
    double power;
    double p = psi / M_PI + 1.0;

    p *= p;
    p *= p;
    p *= p;
    p *= p;
    p *= p;

    power = m_factor_step * m_factor_spring * p;

    uv.x *= power;
    uv.y *= power;
    uv.z *= power;
}

void
rinne::get_repulse_vec(rn_vec &uv, double psi)
{
    double power;
    double p = psi + SQRTPI;

    power = - m_factor_step * m_factor_repulse / (p * p);

    uv.x *= power;
    uv.y *= power;
    uv.z *= power;
}

void
rinne::get_uv_vec(rn_vec &v, const rn_pos &a, const rn_pos &b)
{
    rn_vec va, vb;
    double t;

    TO_RECTANGULAR(va, a, 1.0);
    TO_RECTANGULAR(vb, b, 1.0);

    t = 1.0 - va.x * vb.x - va.y * vb.y - va.z * vb.z;

    v.x = vb.x + va.x * t;
    v.y = vb.y + va.y * t;
    v.z = vb.z + va.z * t;

    v.x -= va.x;
    v.y -= va.y;
    v.z -= va.z;

    NORMALIZE(v);
}

void
rinne::get_uv_vec_rand(rn_vec &v, const rn_pos &a)
{
    static int i = 0, j = 0;
    static double theta[7] = {0.0,
                              M_PI / 7.0,
                              2 * M_PI / 7.0,
                              3 * M_PI / 7.0,
                              4 * M_PI / 7.0,
                              5 * M_PI / 7.0,
                              6 * M_PI / 7.0};
    static double phi[11] = {0.0,
                             M_PI / 11.0,
                             2 * M_PI / 11.0,
                             3 * M_PI / 11.0,
                             4 * M_PI / 11.0,
                             5 * M_PI / 11.0,
                             6 * M_PI / 11.0,
                             7 * M_PI / 11.0,
                             8 * M_PI / 11.0,
                             9 * M_PI / 11.0,
                             10 * M_PI / 11.0};

    for (;;) {
        rn_pos b;
        b.theta = theta[i++];
        b.phi   = phi[j++];

        if (i >= 7)
            i = i % 7;
        if (j >= 11)
            j = j % 11;

        double psi;
        psi = acos(cos(a.theta) * cos(b.theta) +
                   sin(a.theta) * sin(b.theta) *
                   cos(a.phi - b.phi));

        if (isnan(psi))
            continue;

        get_uv_vec(v, a, b);
        get_repulse_vec(v, 0.0001);

        break;
    }
}

void
rinne::force_directed()
{
    if (m_num_node < 2)
        return;

    rn_pos *p_pos = new rn_pos[m_num_node];
    rn_pos *pos_idx = p_pos;

    for (rn_node *p1 = m_node; p1 != &m_node[m_num_node]; p1++) {
        rn_node *p2;
        rn_vec v1 = {0.0, 0.0, 0.0};
        rn_vec v2;
        double cos_theta_a, sin_theta_a;
        double psi;

        cos_theta_a = cos(p1->pos.theta);
        sin_theta_a = sin(p1->pos.theta);

        for (p2 = m_node; p2 != &m_node[m_num_node]; p2++) {
            if (p1 == p2)
                continue;

            psi = acos(cos_theta_a * cos(p2->pos.theta) +
                       sin_theta_a * sin(p2->pos.theta) *
                       cos(p1->pos.phi - p2->pos.phi));

            if (isnan(psi)) {
                get_uv_vec_rand(v2, p1->pos);
            } else {
                get_uv_vec(v2, p1->pos, p2->pos);
                get_repulse_vec(v2, psi);
            }

            v1.x += v2.x;
            v1.y += v2.y;
            v1.z += v2.z;
        }


        rn_edge *p_edge;
        for (p_edge = p1->edge; p_edge != NULL; p_edge = p_edge->next) {
            rn_vec v3;

            psi = acos(cos_theta_a * cos(p_edge->dst->pos.theta) +
                       sin_theta_a * sin(p_edge->dst->pos.theta) *
                       cos(p1->pos.phi - p_edge->dst->pos.phi));

            if (isnan(psi))
                continue;

            get_uv_vec(v3, p1->pos, p_edge->dst->pos);
            get_spring_vec(v3, psi);

            v1.x += v3.x;
            v1.y += v3.y;
            v1.z += v3.z;
        }

        for (p_edge = p1->bp_edge; p_edge != NULL; p_edge = p_edge->bp_next) {
            rn_vec v3;

            psi = acos(cos_theta_a * cos(p_edge->src->pos.theta) +
                       sin_theta_a * sin(p_edge->src->pos.theta) *
                       cos(p1->pos.phi - p_edge->src->pos.phi));

            if (isnan(psi))
                continue;

            get_uv_vec(v3, p1->pos, p_edge->src->pos);
            get_spring_vec(v3, psi);

            v1.x += v3.x;
            v1.y += v3.y;
            v1.z += v3.z;
        }


        rn_vec pvec, cross;
        double rad, norm;

        TO_RECTANGULAR(pvec, p1->pos, 1.0);
        DISTANCE2(rad, v1);

        rad = sqrt(rad);
        if (rad > M_PI_4)
            rad = M_PI_4;

        CROSS_PRODUCT(cross, pvec, v1);
        DISTANCE2(norm, cross);

        norm = 1.0 / sqrt(norm);

        cross.x *= norm;
        cross.y *= norm;
        cross.z *= norm;

        ROTATE(pvec, cross, rad);

        *pos_idx = p1->pos;
        TO_SPHERICAL(*pos_idx, pvec);
        pos_idx++;
    }

    for (int i = 0; i < m_num_node; i++) {
        m_node[i].pos = p_pos[i];
    }

    delete p_pos;
}

void
rinne::init_pos()
{
    srand(time(NULL));
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        p->pos.theta = M_PI_2 * ((double)rand() / RAND_MAX) + M_PI_4;
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
        p_edge->bp_next = p_edge->dst->bp_edge;
        p_edge->dst->bp_edge = p_edge;
        p_edge->src->num_edge++;
        p_edge->dst->num_bp_edge++;

        if (m_max_out_degree < p_edge->src->num_edge)
            m_max_out_degree = p_edge->src->num_edge;

        if (m_max_in_degree < p_edge->dst->num_bp_edge)
            m_max_in_degree = p_edge->dst->num_bp_edge;

        p_edge++;
    }

    init_pos();

    std::cout << "#node = " << m_num_node << std::endl;
    std::cout << "#edge = " << m_num_edge << std::endl;

    m_factor_step /= m_num_node * 100;

    boost::thread th(&run);
    thread = move(th);
}

void
rinne::get_color(double &g, double &b, double &alpha,
                 double min_g, double max_g,
                 double min_b, double max_b,
                 double min_alpha, double max_alpha, double cycle)
{
    double diff_g = max_g - min_g;
    double diff_b = max_b - min_b;
    double diff_alpha = max_alpha - min_alpha;
    double t, r;
    timeval tv;

    cycle *= 0.5;

    gettimeofday(&tv, NULL);

    t = (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;
    t -= m_init_sec;

    r = sin(M_PI * (t - M_PI * 0.5) / cycle) * 0.5 + 0.5;

    g = r * diff_g + min_g;
    b = r * diff_b + min_b;
    alpha = r * diff_alpha + min_alpha;
}

int
main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " graph.dot" << std::endl;
        return 1;
    }

    std::cout << "loading " << argv[1] << " ..." << std::endl;
    rinne_inst.read_dot(argv[1]);

    init_glut(argc, argv);

    return 0;
}
