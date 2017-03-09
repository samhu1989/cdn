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
        typedef std::vector<EdgeX>::iterator Iter;
        typedef std::vector<EdgeX>::const_iterator CIter;
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
        EdgeX* _dual_edge;
    };
    class NodeX{
    public:
        typedef std::vector<NodeX>::iterator Iter;
        typedef std::vector<NodeX>::const_iterator CIter;
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
    float getObjFromX(void);
    void computeD();
    void updateX();
    void updateIX();
    void resetXtoIX();
protected:
    bool isConverge();
    bool isBetterIX();
    //lambda
    float _lambda;
    float _alpha;//coeff for sqaure
    //last obj float
    float _objf_last;
    //last obj integer
    uint32_t _obji_last;
    float _inf;
    float _step;
private:
    //result text
    std::string _res_str;
    std::stringstream  _res_txt;
    //server cost
    float _sp;
    //node vector
    std::vector<NodeX> _x;
    float _sum_cx;
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
