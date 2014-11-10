#ifndef RINNE_HPP
#define RINNE_HPP

#include <sys/time.h>

#include <string>

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
};

struct rn_node {
    rn_pos   pos;
    rn_edge *edge;
    rn_edge *bp_edge;
    int      num_edge;
    int      num_bp_edge;
};

class rinne {
public:
    void read_dot(char *path);
    rinne() : m_is_mouse_down(false),
              m_is_fullscreen(false),
              m_rotate_z(0.0),
              m_rotate_x(0.0),
              m_is_blink(0),
              m_max_in_degree(0),
              m_max_out_degree(0),
              m_top_n(20),
              m_top_idx(0),
              m_factor_repulse(0.01),
              m_factor_spring(0.01),
              m_factor_step(1.0),
              m_cycle(10.0)
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        m_init_sec = (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;
    }

    void on_mouse_down(int button, int x, int y);
    void on_mouse_up(int button, int x, int y);
    void on_mouse_move(int x, int y);
    void on_keyboard(unsigned char key, int x, int y);
    void on_resize(int w, int h);
    void force_directed();
    void reduce_step() { m_factor_step *= 0.5; }
    void init_glui();

    void display();

private:
    bool m_is_mouse_down;
    bool m_is_fullscreen;
    int  m_mouse_x;
    int  m_mouse_y;
    int  m_window_w;
    int  m_window_h;

    double m_rotate_z;
    double m_rotate_x;

    int m_is_blink;

    int m_num_node;
    int m_num_edge;

    int m_max_in_degree;
    int m_max_out_degree;

    rn_node *m_node;
    rn_edge *m_edge;
    std::string *m_label;

    rn_node **m_node_top;
    int    m_top_n;
    int    m_top_idx;

    double m_factor_repulse;
    double m_factor_spring;
    double m_factor_step;
    double m_init_sec;
    double m_cycle;

    void init_pos();
    void draw_node();
    void draw_edge(double g, double b, double alpha);
    void draw_label();
    void draw_tau();
    void get_top_n();
    void update_time();
    void get_uv_vec(rn_vec &v, const rn_pos &a, const rn_pos &b);
    void get_uv_vec_rand(rn_vec &v, const rn_pos &a);
    void get_repulse_vec(rn_vec &uv, double psi);
    void get_spring_vec(rn_vec &uv, double psi);
    void get_color(double &g, double &b, double &alpha,
                   double min_b, double max_b,
                   double min_g, double max_g,
                   double min_alpha, double max_alpha);
};

#endif // RINNE_HPP
