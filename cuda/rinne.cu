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
#include <boost/bind.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                              boost::property<boost::vertex_name_t,
                                              std::string> > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::edge_iterator edge_iter;

#define TIMERSEC 16
#define SQRTPI 0.5641898
#define CAMERA_Y -3.0
#define NODE_R_MAX 0.05
#define NODE_R_MIN 0.003
#define NODE_R_DIFF (NODE_R_MAX - NODE_R_MIN)
#define NODE_MAX_G 0.7
#define NODE_MIN_G 0.5
#define NODE_MAX_B 0.6
#define NODE_MIN_B 0.2
#define EDGE_MAX_G 0.5
#define EDGE_MIN_G 0.06
#define EDGE_MAX_B 0.4
#define EDGE_MIN_B 0.0
#define EDGE_MAX_ALPHA 1.0
#define EDGE_MIN_ALPHA 0.15
#define EDGE_LINES 6
#define LABEL_MAX_G 1.0
#define LABEL_MIN_G 0.4
#define LABEL_MAX_B 0.8
#define LABEL_MIN_B 0.2

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void
gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code),
                file, line);
        if (abort) exit(code);
    }
}
#define DISTANCE2(D, A) do {                    \
        float x2, y2, z2;                       \
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
        float tmp = (A).x * (A).x + (A).y * (A).y;      \
        float r   = sqrt(tmp + (A).z * (A).z);          \
        float rxy = sqrt(tmp);                          \
        if (r > 0.0) {                                  \
            (D).theta = acosf((A).z / r);               \
            (D).phi = acosf((A).x / rxy);               \
            if ((A).y < 0.0) {                          \
                (D).phi = 2 * M_PI - (D).phi;           \
            }                                           \
        } else {                                        \
            (D).theta = 0.0;                            \
            (D).phi   = 0.0;                            \
        }                                               \
    } while (0)

#define TO_RECTANGULAR(D, A, R) do {             \
        float sin_theta = sinf((A).theta);       \
        (D).x = (R) * sin_theta * cosf((A).phi); \
        (D).y = (R) * sin_theta * sinf((A).phi); \
        (D).z = (R) * cos((A).theta);            \
    } while (0)

#define GET_UV(U, V, A) do {                   \
        float cos_theta = cosf((A).theta);     \
        float cos_phi   = cosf((A).phi);       \
        float sin_phi   = sinf((A).phi);       \
        (U).x = - cos_theta * cos_phi;         \
        (U).y = - cos_theta * sin_phi;         \
        (U).z = sinf(A.theta);                 \
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
        float r2 = (RAD) * -0.5;                \
        float sin_rad2 = sinf(r2);              \
        float cos_rad2 = cosf(r2);              \
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
        float d = sqrtf((V).x * (V).x + (V).y * (V).y + (V).z * (V).z); \
        if (d > 0.0001) {                                               \
            d = 1.0 / d;                                                \
            (V).x *= d;                                                 \
            (V).y *= d;                                                 \
            (V).z *= d;                                                 \
        } else {                                                        \
            (V).x = (V).y = (V).z = 0.0;                                \
        }                                                               \
    } while (0)

#define TO_CUDA_MEM(D, ADDRCPU, HCPU, HCUDA) do {       \
            if ((ADDRCPU) == NULL) {                    \
                (D) = NULL;                             \
            } else {                                    \
                (D) = (HCUDA) + ((ADDRCPU) - (HCPU));   \
            }                                           \
        } while (0);

rinne rinne_inst;

__device__ __constant__ int num_node_cuda;
__device__ __constant__ float factor_repulse_cuda;
__device__ __constant__ float factor_step_cuda;
__device__ __constant__ float factor_spring_cuda;

__global__ void force_directed(rn_node *p_node, rn_pos *p_pos);

void
render_string(float x, float y, float z, std::string const& str)
{
    int len;
    
    glRasterPos3f(x, y, z);

    len = str.size();

    for (int i = 0; i < len; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str.at(i));
    }
}

void
run()
{
    int num_node = rinne_inst.get_num_node();

    if (num_node < 2 )
        return;

    int block_size;
    int min_grid_size;
    int grid_size;
    int total_thread;

    gpuErrchk(cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                                 force_directed, 0, num_node));

    grid_size = (num_node + block_size - 1) / block_size;
    total_thread = grid_size * block_size;

    float factor_repulse = rinne_inst.get_factor_repulse();
    float factor_step = rinne_inst.get_factor_step();
    float factor_spring = rinne_inst.get_factor_spring();

    gpuErrchk(cudaMemcpyToSymbol(num_node_cuda, &num_node,
                                 sizeof(num_node_cuda)));
    gpuErrchk(cudaMemcpyToSymbol(factor_repulse_cuda, &factor_repulse,
                                 sizeof(factor_repulse_cuda)));
    gpuErrchk(cudaMemcpyToSymbol(factor_step_cuda, &factor_step,
                                 sizeof(factor_step_cuda)));
    gpuErrchk(cudaMemcpyToSymbol(factor_spring_cuda, &factor_spring,
                                 sizeof(factor_spring_cuda)));

    std::cout << "grid size = " << grid_size
              << "\nblock size = " << block_size
              << "\ntotal thread = " << total_thread << std::endl;

    for (int i = 0; i < 200; i++) {
        force_directed<<<grid_size, block_size>>>(rinne_inst.get_node_cuda(),
                                                  rinne_inst.get_pos_cuda());
        gpuErrchk(cudaDeviceSynchronize());
        rinne_inst.copy_result();

        std::cout << i << std::endl;

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
rinne::update_time()
{
    double  t, diff, r;
    double  cycle = m_cycle * 0.5;
    timeval tv;
    
    gettimeofday(&tv, NULL);

    t = (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;

    m_prev_sec = m_current_sec;
    m_current_sec = t;

    diff = t - m_init_sec;

    r = sin(M_PI * (diff - M_PI * 0.5) / cycle) * 0.5 + 0.5;

    if (diff > cycle && r < 0.004) {
        m_init_sec = t;
        m_top_idx++;
        if (m_top_idx > m_top_n)
            m_top_idx = 0;
    }
}

void
rinne::rotate_view()
{
    if (m_is_auto_rotate && m_top_idx  > 0) {
        float theta = m_node_top[m_top_idx - 1]->pos.theta + M_PI_2;
        float diff, tmp;

        theta += m_rotate_x * 2 * M_PI;

        if (theta > 2 * M_PI) {
            theta -= 2 * M_PI;
        }

        diff = M_PI - theta;

        if (fabs(diff) > 0.05) {
            if (diff > 0.0) {
                m_rotate_x += (m_current_sec - m_prev_sec) / (m_cycle * 4.0);
            } else {
                m_rotate_x -= (m_current_sec - m_prev_sec) / (m_cycle * 4.0);
            }

            m_rotate_x = modf(m_rotate_x, &tmp);
            if (m_rotate_x < 0.0) {
                m_rotate_x += 1.0;
            }
        }

        float phi = m_node_top[m_top_idx - 1]->pos.phi;

        phi += m_rotate_z * 2 * M_PI;

        if (phi > 2 * M_PI) {
            phi -= 2 * M_PI;
        }

        diff = 3 * M_PI_2 - phi;
        if (fabs(diff) > 0.05) {
            if (0 < diff && diff < M_PI) {
                m_rotate_z += (m_current_sec - m_prev_sec) / (m_cycle * 4.0);
            } else {
                m_rotate_z -= (m_current_sec - m_prev_sec) / (m_cycle * 4.0);
            }

            m_rotate_z = modf(m_rotate_z, &tmp);
            if (m_rotate_z < 0.0) {
                m_rotate_z += 1.0;
            }
        }
    }
}

void
rinne::display()
{
    update_time();
    rotate_view();

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h, 0.1, 8);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();

    glRotatef(360 * m_rotate_x, 1.0, 0.0, 0.0);
    glRotatef(360 * m_rotate_z, 0.0, 0.0, 1.0);

    glColor3f(0.4, 0.4, 0.4);
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
    rinne_inst.on_keyboard(key, x, y);
}

void
rinne::on_keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case 'f':
    case 'F':
        if (m_is_fullscreen) {
            glutPositionWindow(0,0);
            glutReshapeWindow(1200, 900);
        } else {
            glutFullScreen();
        }
        m_is_fullscreen = ! m_is_fullscreen;
        break;
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
    if (m_is_mouse_down && ! m_is_auto_rotate) {
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
rinne::draw_label()
{
    double g, b, alpha;
    rn_vec v;

    for (int i = 0; i < m_top_n; i++) {
        if (m_is_blink && i + 1 == m_top_idx)
            continue;

        TO_RECTANGULAR(v, m_node_top[i]->pos, 1.0);

        glColor3f(0.0, 0.0, 0.0);
        render_string(v.x - 0.005, v.y, v.z - 0.005,
                      m_label[(m_node_top[i] - m_node)]);

        get_color(g, b, alpha, LABEL_MIN_G, LABEL_MAX_G,
                  LABEL_MIN_B, LABEL_MAX_B, 0.0, 0.0);

        if (m_is_blink) {
            if (m_top_idx == 0) {
                glColor3f(0.0, g, b);
            } else{
                glColor3f(0.0, LABEL_MIN_G, LABEL_MIN_B);
            }
        } else {
            glColor3f(0.0, LABEL_MAX_G, LABEL_MAX_B);
        }

        render_string(v.x, v.y, v.z, m_label[(m_node_top[i] - m_node)]);
    }

    if (m_is_blink && m_top_idx > 0) {
        TO_RECTANGULAR(v, m_node_top[m_top_idx - 1]->pos, 1.0);
        
        glColor3f(0.0, 0.0, 0.0);
        render_string(v.x - 0.005, v.y, v.z - 0.005,
                      m_label[(m_node_top[m_top_idx - 1] - m_node)]);

        get_color(g, b, alpha, LABEL_MIN_G, LABEL_MAX_G,
                  LABEL_MIN_B, LABEL_MAX_B, 0.0, 0.0);
        glColor3f(0.0, g, b);
        render_string(v.x, v.y, v.z,
                      m_label[(m_node_top[m_top_idx - 1] - m_node)]);
    }
}

void
rinne::draw_edge(double g, double b, double alpha)
{
    rn_node *dst = m_node_top[m_top_idx - 1];

    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        rn_vec ev, ev1, ev2;
        double cos_theta_a = cos(p->pos.theta);
        double sin_theta_a = sin(p->pos.theta);

        TO_RECTANGULAR(ev, p->pos, 1.0);

        for (rn_edge *p_edge = p->edge; p_edge != NULL; p_edge = p_edge->next) {
            rn_pos delta;

            delta.theta = p->pos.theta - p_edge->dst->pos.theta;
            delta.phi   = p->pos.phi - p_edge->dst->pos.phi;

            if (delta.phi > M_PI) {
                delta.phi = -(2 * M_PI - delta.phi);
            } else if (delta.phi < -M_PI) {
                delta.phi = 2 * M_PI + delta.phi;
            }
            
            if (!m_is_blink || m_top_idx == 0 || dst == p_edge->dst) {
                glColor4f(0.0, g, b, alpha);
            } else {
                glColor4f(0.0, EDGE_MIN_G, EDGE_MIN_B, EDGE_MIN_ALPHA);
            }

            if (m_is_blink && m_top_idx != 0 && dst == p_edge->dst) {
                glLineWidth(2.0f);
            } else {
                glLineWidth(1.0f);
            }

            TO_RECTANGULAR(ev2, p_edge->dst->pos, 1.0);

            double psi = acos(cos_theta_a * cos(p_edge->dst->pos.theta) +
                              sin_theta_a * sin(p_edge->dst->pos.theta) *
                              cos(p->pos.phi - p_edge->dst->pos.phi));

            rn_vec cross;

            ev1 = ev;
            psi /= EDGE_LINES;

            CROSS_PRODUCT(cross, ev, ev2);
            NORMALIZE(cross);
            glBegin(GL_LINE_STRIP);

            glVertex3f(ev.x, ev.y , ev.z);

            for (int i = 0; i < EDGE_LINES - 1; i++) {
                ROTATE(ev1, cross, psi);
                glVertex3f(ev1.x, ev1.y, ev1.z);
            }

            glVertex3f(ev2.x, ev2.y , ev2.z);

            glEnd();
            glLineWidth(1.0f);
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
    double b, g, alpha;

    // draw far side nodes
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h, -CAMERA_Y, 8);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glColor3d(0.0, NODE_MIN_G * 0.125, NODE_MIN_B * 0.125);
    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        rn_vec a;
        double r = p->num_bp_edge * r_denom;

        r = r * NODE_R_DIFF + NODE_R_MIN;

        TO_RECTANGULAR(a, p->pos, 1.0);

        glPushMatrix();

        glTranslatef(a.x, a.y, a.z);
        glutSolidSphere(r, 8, 8);

        glPopMatrix();
    }

    // draw edges
    if (m_is_blink) {
        max_alpha = EDGE_MAX_ALPHA;
        min_alpha = EDGE_MIN_ALPHA;
        max_g = EDGE_MAX_G;
        min_g = EDGE_MIN_G;
        max_b = EDGE_MAX_B;
        min_b = EDGE_MIN_B;

        get_color(g, b, alpha, min_g, max_g, min_b, max_b, min_alpha, max_alpha);
    } else {
        g = EDGE_MAX_G;
        b = EDGE_MAX_B;
        alpha = EDGE_MAX_ALPHA;
    }

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h,
                   0.1, -CAMERA_Y);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    draw_edge(g, b, alpha);


    // draw near side nodes
    if (m_is_blink) {
        max_g = NODE_MAX_G;
        min_g = NODE_MIN_G;
        max_b = NODE_MAX_B;
        min_b = NODE_MIN_B;

        get_color(g, b, alpha, min_g, max_g, min_b, max_b, min_alpha, max_alpha);
    } else {
        g = NODE_MAX_G;
        b = NODE_MIN_G;
    }

    rn_node *dst = m_node_top[m_top_idx - 1];

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)m_window_w / (double)m_window_h,
                   0.1, -CAMERA_Y);
    gluLookAt(0.0, CAMERA_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    for (rn_node *p = m_node; p != &m_node[m_num_node]; p++) {
        rn_vec a;
        double r = p->num_bp_edge * r_denom;

        r = r * NODE_R_DIFF + NODE_R_MIN;

        TO_RECTANGULAR(a, p->pos, 1.0);

        if (!m_is_blink || m_top_idx == 0 || p == dst) {
            glColor3f(0.0, g, b);
        } else {
            glColor3f(0.0, NODE_MIN_G, NODE_MIN_B);
        }

        glPushMatrix();

        glTranslatef(a.x, a.y, a.z);
        glutSolidSphere(r, 8, 8);

        glPopMatrix();
    }

    draw_label();
}

__device__
void
get_spring_vec(rn_vec &uv, float psi)
{
    float power;
    float p = psi / M_PI + 1.0f;

    p *= p;
    p *= p;
    p *= p;
    p *= p;
    p *= p;

    power = factor_step_cuda * factor_spring_cuda * p;

    uv.x *= power;
    uv.y *= power;
    uv.z *= power;
}

__device__
void
get_repulse_vec(rn_vec &uv, float psi)
{
    float power;
    float p = psi + M_PI;

    power = - factor_step_cuda * factor_repulse_cuda / (p * p);

    uv.x *= power;
    uv.y *= power;
    uv.z *= power;
}

__device__
void
get_uv_vec(rn_vec &v, const rn_pos &a, const rn_pos &b)
{
    rn_vec va, vb;
    float t;

    TO_RECTANGULAR(va, a, 1.0);
    TO_RECTANGULAR(vb, b, 1.0);

    t = 1.0f - va.x * vb.x - va.y * vb.y - va.z * vb.z;

    v.x = vb.x + va.x * t;
    v.y = vb.y + va.y * t;
    v.z = vb.z + va.z * t;

    v.x -= va.x;
    v.y -= va.y;
    v.z -= va.z;

    NORMALIZE(v);
}

int
cmp_node(const void *lhs, const void *rhs)
{
    const rn_node *p1 = *(rn_node**)lhs;
    const rn_node *p2 = *(rn_node**)rhs;

    if (p1->num_bp_edge > p2->num_bp_edge)
        return -1;

    if (p1->num_bp_edge < p2->num_bp_edge)
        return 1;

    return 0;
}

void
rinne::get_top_n()
{
    rn_node **p;

    if (m_num_node < m_top_n)
        m_top_n = m_num_node;

    p = new rn_node*[m_num_node];
    m_node_top = new rn_node*[m_top_n];

    for (int i = 0; i < m_num_node; i++) {
        p[i] = &m_node[i];
    }

    qsort(p, m_num_node, sizeof(p), cmp_node);

    for (int j = 0; j < m_top_n; j++) {
        m_node_top[j] = p[j];
    }

    delete[] p;
}

void
rinne::copy_result()
{
    rn_pos  *p_pos = new rn_pos[m_num_node];
    rn_node *p_node = new rn_node[m_num_node];
    
    gpuErrchk(cudaMemcpy(p_pos, m_pos_cuda, sizeof(*p_pos) * m_num_node,
                         cudaMemcpyDeviceToHost));
    gpuErrchk(cudaMemcpy(p_node, m_node_cuda, sizeof(*p_node) * m_num_node,
                         cudaMemcpyDeviceToHost));

    for (int i = 0; i < m_num_node; i++) {
        p_node[i].pos = m_node[i].pos = p_pos[i];
    }
    
    gpuErrchk(cudaMemcpy(m_node_cuda, p_node, sizeof(*p_node) * m_num_node,
                         cudaMemcpyHostToDevice));

    delete[] p_pos;
    delete[] p_node;
}

void
rinne::get_uv_vec_rand(rn_vec &v, const rn_pos &a)
{
    static int i = 0, j = 0;
    static float theta[7] = {0.0,
                             M_PI / 7.0,
                             2 * M_PI / 7.0,
                             3 * M_PI / 7.0,
                             4 * M_PI / 7.0,
                             5 * M_PI / 7.0,
                             6 * M_PI / 7.0};
    static float phi[11] = {0.0,
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

        float psi;
        psi = acosf(cosf(a.theta) * cosf(b.theta) +
                    sinf(a.theta) * sinf(b.theta) *
                    cosf(a.phi - b.phi));

        if (isnan(psi))
            continue;

        //get_uv_vec(v, a, b);
        //get_repulse_vec(v, 0.0001);

        break;
    }
}

__global__
void
force_directed(rn_node *p_node, rn_pos *p_pos)
{
    int id = blockDim.x * blockIdx.x + threadIdx.x;

    rn_node *p1;
    p_pos = &p_pos[id];

    //    if (id < num_node_cuda) {
    for (p1 = &p_node[id]; p1 < p_node + num_node_cuda; p1 += num_node_cuda) {
        rn_node *p2;
        rn_vec v1 = {0.0, 0.0, 0.0};
        rn_vec v2;
        float cos_theta_a, sin_theta_a;
        float psi;

        cos_theta_a = cosf(p1->pos.theta);
        sin_theta_a = sinf(p1->pos.theta);

        for (p2 = p_node; p2 != &p_node[num_node_cuda]; p2++) {
            if (p1 == p2)
                continue;

            psi = acosf(cos_theta_a * cosf(p2->pos.theta) +
                        sin_theta_a * sinf(p2->pos.theta) *
                        cosf(p1->pos.phi - p2->pos.phi));

            get_uv_vec(v2, p1->pos, p2->pos);
            get_repulse_vec(v2, psi);

            v1.x += v2.x;
            v1.y += v2.y;
            v1.z += v2.z;
        }

        rn_edge *p_edge;
        for (p_edge = p1->edge; p_edge != NULL; p_edge = p_edge->next) {
            rn_vec v3;

            psi = acosf(cos_theta_a * cosf(p_edge->dst->pos.theta) +
                        sin_theta_a * sinf(p_edge->dst->pos.theta) *
                        cosf(p1->pos.phi - p_edge->dst->pos.phi));

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

            psi = acosf(cos_theta_a * cosf(p_edge->src->pos.theta) +
                        sin_theta_a * sinf(p_edge->src->pos.theta) *
                        cosf(p1->pos.phi - p_edge->src->pos.phi));

            if (isnan(psi))
                continue;

            get_uv_vec(v3, p1->pos, p_edge->src->pos);
            get_spring_vec(v3, psi);

            v1.x += v3.x;
            v1.y += v3.y;
            v1.z += v3.z;
        }

        rn_vec pvec, cross;
        double rad;

        TO_RECTANGULAR(pvec, p1->pos, 1.0);
        DISTANCE2(rad, v1);

        rad = sqrt(rad);
        if (rad > M_PI_4)
            rad = M_PI_4;

        CROSS_PRODUCT(cross, pvec, v1);
        NORMALIZE(cross);
        ROTATE(pvec, cross, rad);

        rn_pos pos = p1->pos;
        TO_SPHERICAL(pos, pvec);
        *p_pos = pos;
        p_pos += num_node_cuda;
    }
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
rinne::init_graph_cuda()
{
    int i;
    rn_node *p_node = new rn_node[m_num_node];
    rn_edge *p_edge = new rn_edge[m_num_edge];

    gpuErrchk(cudaMalloc((void**)&m_node_cuda,
                         sizeof(*m_node_cuda) * m_num_node));
    gpuErrchk(cudaMalloc((void**)&m_edge_cuda,
                         sizeof(*m_edge_cuda) * m_num_edge));
    gpuErrchk(cudaMalloc((void**)&m_pos_cuda,
                         sizeof(*m_pos_cuda) * m_num_node));

    for (i = 0; i < m_num_node; i++) {
        p_node[i].pos = m_node[i].pos;
        p_node[i].num_edge = m_node[i].num_edge;
        p_node[i].num_bp_edge = m_node[i].num_bp_edge;

        TO_CUDA_MEM(p_node[i].edge, m_node[i].edge, m_edge, m_edge_cuda);
        TO_CUDA_MEM(p_node[i].bp_edge, m_node[i].bp_edge, m_edge, m_edge_cuda);
    }

    for (i = 0; i < m_num_edge; i++) {
        TO_CUDA_MEM(p_edge[i].src, m_edge[i].src, m_node, m_node_cuda);
        TO_CUDA_MEM(p_edge[i].dst, m_edge[i].dst, m_node, m_node_cuda);
        TO_CUDA_MEM(p_edge[i].next, m_edge[i].next, m_edge, m_edge_cuda);
        TO_CUDA_MEM(p_edge[i].bp_next, m_edge[i].bp_next, m_edge, m_edge_cuda);
    }

    gpuErrchk(cudaMemcpy(m_node_cuda, p_node, sizeof(*p_node) * m_num_node,
                         cudaMemcpyHostToDevice));
    gpuErrchk(cudaMemcpy(m_edge_cuda, p_edge, sizeof(*p_edge) * m_num_edge,
                         cudaMemcpyHostToDevice));

    delete[] p_node;
    delete[] p_edge;
}

void
rinne::read_dot(char *path)
{
    std::ifstream file(path);

    Graph g;
    boost::dynamic_properties dp(boost::ignore_other_properties);
    dp.property("vertex_name", get(boost::vertex_name, g));

    boost::read_graphviz(file, g, dp, "vertex_name");

    m_num_node = num_vertices(g);
    m_num_edge = num_edges(g);

    m_node  = new rn_node[m_num_node];
    m_edge  = new rn_edge[m_num_edge];
    m_label = new std::string[m_num_node];

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
    
    std::pair<vertex_iter, vertex_iter> vp;
    int i = 0;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first) {
        m_label[i++] = get(boost::vertex_name, g, *vp.first);
    }

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

    get_top_n();
    init_graph_cuda();

    run();
}

void
rinne::get_color(double &g, double &b, double &alpha,
                 double min_g, double max_g,
                 double min_b, double max_b,
                 double min_alpha, double max_alpha)
{
    double diff_g = max_g - min_g;
    double diff_b = max_b - min_b;
    double diff_alpha = max_alpha - min_alpha;
    double diff, r, cycle;

    cycle = m_cycle * 0.5;

    diff = m_current_sec - m_init_sec;

    r = sin(M_PI * (diff - M_PI * 0.5) / cycle) * 0.5 + 0.5;

    //std::cout << r << std::endl;

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
