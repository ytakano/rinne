#include "rinne.hpp"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
        float x2, y2, z2;                       \
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

#define TO_SPHERICAL(D, A, R) do {              \
        float d = sqrtf(A.x * A.x + A.y * A.y); \
        if (d > 0.0f) {                         \
            D.phi = acosf(A.x / d);             \
        } else {                                \
            D.phi = 0.0f;                       \
        }                                       \
        D.theta = acosf(A.z / R);               \
    } while (0)

#define TO_RECTANGULAR(D, A, R) do {            \
        float sin_theta = sinf(A.theta);        \
        D.x = R * sin_theta * cosf(A.phi);      \
        D.y = R * sin_theta * sinf(A.phi);      \
        D.z = R * cosf(A.theta);                \
    } while (0)

#define QUATERNION_MUL(D, A, B) do {                            \
        D.w = A.w * B.w - A.i * B.i - A.j * B.j - A.k * B.k;    \
        D.i = A.i * B.w + A.w * B.i - A.k * B.j + A.j * B.k;    \
        D.j = A.j * B.w + A.k * B.i + A.w * B.j - A.i * B.k;    \
        D.k = A.k * B.w - A.j * B.i + A.i * B.j + A.w * B.k;    \
    } while (0)

#define ROTATE(A, V, RAD) do {                  \
        rn_quaternion p, q, r;                  \
        float r2 = RAD * -0.5f;                 \
        float sin_rad2 = sinf(r2);              \
        float cos_rad2 = cosf(r2);              \
                                                \
        p.w = 0.0f;                             \
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
        float d = sqrtf(V.x * V.x + V.y * V.y + V.z * V.z);     \
        if (d > 0.0f) {                                         \
            V.x /= d;                                           \
            v.y /= d;                                           \
            v.z /= d;                                           \
        } else {                                                \
            v.x = v.y = v.z = 0.0f;                             \
        }                                                       \
    } while (0)

rn_node *node_pool;
rn_edge *edge_pool;

rinne rinne_inst;

void
cpu_rotate(rn_vec &a, const rn_vec &v, float rad)
{
    ROTATE(a, v, rad);
}

void
display()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush();

    glColor3d(0.4, 0.4, 0.4);
    glutWireSphere(1.0, 16, 16);

    glutSwapBuffers();
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
init_glut(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(display);
    glutKeyboardFunc(on_keyboard);
    glutReshapeFunc(on_resize);
    glutFullScreen();
    glutMainLoop();
}

void
rinne::cpu_spring_v0()
{

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

    std::cout << "loading " << argv[0] << " ..." << std::endl;
    rinne_inst.read_dot(argv[1]);

    init_glut(argc, argv);

    return 0;
}
