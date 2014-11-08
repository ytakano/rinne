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
    rn_edge *bp_next;
    bool     is_bidirection;
};

struct rn_node {
    rn_pos   pos;
    rn_edge *edge;
    rn_edge *bp_edge;
    int      num_edge;
};

class rinne {
public:
    void read_dot(char *path);
    rinne() : m_is_mouse_down(false),
              m_rotate_z(0.0),
              m_rotate_x(0.0),
              m_factor_repulse(0.01),
              m_factor_spring(0.01),
              m_factor_step(1.0) { }

    void on_mouse_down(int button, int x, int y);
    void on_mouse_up(int button, int x, int y);
    void on_mouse_move(int x, int y);
    void force_directed();

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

    double m_factor_repulse;
    double m_factor_spring;
    double m_factor_step;

    void init_pos();
    void draw_node();
    void draw_tau();
    void get_uv_vec(rn_vec &v, const rn_pos &a, const rn_pos &b);
    void get_uv_vec_rand(rn_vec &v, const rn_pos &a);
    void get_repulse_vec(rn_vec &uv, double psi);
    void get_spring_vec(rn_vec &uv, double psi);
};

#endif // RINNE_HPP
