#ifndef RINNE_HPP
#define RINNE_HPP

struct rn_quaternion {
    float w;
    float i, j, k;
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
};

struct rn_node {
    rn_pos   pos;
    rn_edge *edge;
    int      num_edge;
};

class rinne {
public:
    void read_dot(char *path);
    rinne() : m_coulomb(1) { }

private:
    int m_num_node;
    int m_num_edge;

    rn_node *m_node;
    rn_edge *m_edge;

    float m_coulomb; // クーロン力の定数

    void cpu_spring_v0();
};

#endif // RINNE_HPP
