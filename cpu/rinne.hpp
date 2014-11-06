#ifndef RINNE_HPP
#define RINNE_HPP

struct rn_quaternion {
    double w;
    double i, j, k;
};

struct rn_uv {
    double u, v;
};

struct rn_vec {
    double x, y, z;
};

struct rn_pos {
    double theta; // latitude, 0 ... pi
    double phi;   // longitude, 0 ... 2 pi
};

struct rn_node;

struct rn_edge {
    rn_node *src;
    rn_node *dst;
    rn_edge *next;
};

struct rn_node {
    rn_pos   pos;
    rn_edge *edge;
    int      num_edge;
};

class rinne {
public:
    void read_dot(char *path);
    rinne() : m_is_mouse_down(false), m_coulomb(1),
              m_rotate_z(0.0f), m_rotate_x(0.0f) { }

    void on_mouse_down(int button, int x, int y);
    void on_mouse_up(int button, int x, int y);
    void on_mouse_move(int x, int y);

    void display();

private:
    bool m_is_mouse_down;
    int  m_mouse_x;
    int  m_mouse_y;

    double m_rotate_z;
    double m_rotate_x;

    int m_num_node;
    int m_num_edge;

    rn_node *m_node;
    rn_edge *m_edge;

    double m_coulomb; // クーロン力の定数

    void init_pos();
    void draw_node();
    void draw_tau();
    void cpu_spring_v0();
};

#endif // RINNE_HPP
