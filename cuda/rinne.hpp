#ifndef RINNE_HPP
#define RINNE_HPP

#include <pthread.h>

#include <sys/time.h>

#include <string>

struct rn_quaternion {
    float w;
    float i, j, k;
};

struct rn_uv {
    float u, v;
};

struct rn_vec {
    float x, y, z;
};

struct rn_pos {
    float theta; // latitude, 0 ... pi
    float phi;   // longitude, 0 ... 2 pi
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
              m_is_blink(1),
              m_is_auto_rotate(1),
              m_max_in_degree(0),
              m_max_out_degree(0),
              m_top_n(50),
              m_top_idx(1),
              m_factor_repulse(0.1),
              m_factor_spring(0.01),
              m_factor_step(1.0),
              m_cycle(30.0),
              m_score(0.0),
              m_score_loop(0)
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        m_init_sec = (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;
        m_prev_sec = m_current_sec = m_init_sec;
    }

    void on_mouse_down(int button, int x, int y);
    void on_mouse_up(int button, int x, int y);
    void on_mouse_move(int x, int y);
    void on_keyboard(unsigned char key, int x, int y);
    void on_resize(int w, int h);
    void on_menu(int id);
    void reduce_step() { m_factor_step *= 0.5; }
    void copy_result();
    void free_mem();
    void inc_loop() { m_score_loop++; }
    int  get_num_node() { return m_num_node; }
    int  get_num_edge() { return m_num_edge; }
    rn_node* get_node_cuda() { return m_node_cuda; }
    rn_pos*  get_pos_cuda() { return m_pos_cuda; }
    float get_factor_repulse() { return m_factor_repulse; }
    float get_factor_step() { return m_factor_step; }
    float get_factor_spring() { return m_factor_spring; }
    int   get_window_h() { return m_window_h; }
    int   get_window_w() { return m_window_w; }
    void  set_score(double score) { m_score = score; }

    void display();

private:
    pthread_t m_thread;

    bool m_is_mouse_down;
    bool m_is_fullscreen;
    int  m_mouse_x;
    int  m_mouse_y;
    int  m_window_w;
    int  m_window_h;

    double m_rotate_z;
    double m_rotate_x;

    int m_is_blink;
    int m_is_auto_rotate;

    int m_num_node;
    int m_num_edge;

    int m_max_in_degree;
    int m_max_out_degree;

    rn_node *m_node;
    rn_edge *m_edge;
    rn_node *m_node_cuda;
    rn_edge *m_edge_cuda;
    rn_pos  *m_pos_cuda;
    std::string *m_label;

    rn_node **m_node_top;
    int    m_top_n;
    int    m_top_idx;

    float m_factor_repulse;
    float m_factor_spring;
    float m_factor_step;
    double m_init_sec;
    double m_current_sec;
    double m_prev_sec;
    double m_cycle;
    double m_score;
    int    m_score_loop;

    void init_graph_cuda();
    void init_pos();
    void draw_node();
    void draw_edge(double g, double b, double alpha);
    void draw_label();
    void draw_status();
    void get_top_n();
    void update_time();
    void rotate_view();
    void get_uv_vec_rand(rn_vec &v, const rn_pos &a);
    void get_color(double &g, double &b, double &alpha,
                   double min_b, double max_b,
                   double min_g, double max_g,
                   double min_alpha, double max_alpha);
};

#endif // RINNE_HPP
