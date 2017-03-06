#ifndef __ROUTE_H__
#define __ROUTE_H__
#include "lib_io.h"
#include <iostream>
#include <ctime>
#include <sstream>
#include <vector>
#define M_PI        3.14159265358979323846
class CostLess
{
public:
    CostLess(char * topo[MAX_EDGE_NUM], int line_num);
    ~CostLess();
    uint32_t less();
    bool isEnd();
    const char* getRes();
protected:
    class EdgeX{
    public:
        EdgeX(uint32_t i,uint32_t j,float x,float dx,float xm,float cost):
            _i(i),_j(j),_x(x),_dx(dx),_x_max(xm),_a(cost)
        {
            _ix = 0;
        }
        uint32_t _i; //outgoing node
        uint32_t _j; //incoming node
        float _x; //net traffic
        uint32_t _ix;//integer net traffic
        float _dx;//net traffic derivation
        float _x_max;//net traffic limit
        float _a; //net cost
    };
    class NodeX{
    public:
        NodeX(
                uint32_t n,
                float x,
                float dx,
                int32_t c,
                float cx
        ):_n(n),_x(x),_dx(dx),_ci(c),_cx(cx)
        {
            _ix = 0;
            _out_edge.reserve(20);//reserve accoding to maximum number of the edge
        }
        uint32_t _n;
        float _x;//server traffic
        uint32_t _ix;//integer server traffic
        float _dx;//server derivation
        int32_t _ci;//consumer id
        float _cx;//consumer traffic
        std::vector<EdgeX> _out_edge; //outgoing edges
    };
    void init(char* topo[MAX_EDGE_NUM],int line_num);
    float computeObj();
    void computeD();
    void projectD();
    void update();
private:
//result text
std::stringstream _res_txt;
//server cost
float sp_;
//node vector
std::vector<NodeX> _x;
};

class Timer
{
public:
    static void tic(void)
    {
        _time_start = std::time(nullptr);
    }
    static std::time_t toc(void)
    {
        return ( std::time(nullptr) - _time_start );
    }
    static bool timeout(void)
    {
        return ( ( std::time(nullptr) - _time_start ) > _time_max );
    }
private:
    //start time
    static std::time_t _time_start;
    //max time
    static const std::time_t _time_max = 85;
};

void deploy_server(char * graph[MAX_EDGE_NUM], int edge_num, char * filename);
#endif
