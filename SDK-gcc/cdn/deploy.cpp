#include "deploy.h"
#include <stack>
#include <cassert>
#include <cmath>
#include <limits>
#include <cstring>
#include "simplex.h"
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif
std::time_t Timer::_time_start;

CostLess::CostLess(char * topo[MAX_EDGE_NUM], int line_num):_iter_cnt(0)
{
    init(topo,line_num);
}

CostLess::~CostLess()
{

}

void CostLess::init(char* topo[MAX_EDGE_NUM],int line_num)
{
    PRINT("CostLess::init\n");
    std::stringstream head(topo[0]);
    uint32_t node_n,edge_n,c_n;
    PRINT("CostLess::init: get node_n edge_n c_n\n");
    head >> node_n >> edge_n >> c_n;
    PRINT("CostLess::init: node_n=%u edge_n=%u c_n=%u\n",node_n,edge_n,c_n);
    //reserve for nodes
    _x.reserve(node_n);
    //create nodes
    for(uint32_t n_i = 0 ; n_i < node_n ; ++n_i )
    {
        _x.emplace_back(n_i,0.0f,0.0f,-1,0.0f);
    }
    PRINT("CostLess::init: get server price\n");
    std::stringstream price(topo[2]);
    uint32_t sp;
    price >> sp;
    _sp = sp;
    PRINT("CostLess::init: _sp=%u\n",uint32_t(_sp));
    PRINT("CostLess::init: get link information\n");
    for( uint32_t link_i = 0; link_i < edge_n ; ++link_i )
    {
        std::stringstream link_txt( topo[4+link_i] );
        uint32_t s,e,max_band,cost;
        link_txt >> s >> e >> max_band >> cost;
        _x[s]._out_edge.emplace_back(s,e,0.0f,0.0f,float(max_band),float(cost));
        _x[e]._out_edge.emplace_back(e,s,0.0f,0.0f,float(max_band),float(cost));
        _x[s]._out_edge.back()._dual_edge = & _x[e]._out_edge.back();
        _x[e]._out_edge.back()._dual_edge = & _x[s]._out_edge.back();
    }
    PRINT("CostLess::init: get client information\n");
    for( uint32_t c_i = 0; c_i < c_n; ++c_i)
    {
        std::stringstream c_txt( topo[4+edge_n+1+c_i] );
        uint32_t c_id,net_id,c_traffic;
        c_txt >> c_id >> net_id >> c_traffic;
        NodeX& node = _x[net_id];
        node._ci = c_id;
        node._cx = c_traffic;
        node._x = c_traffic; //init the node result by placing a server at the connected node
        node._ix = c_traffic;
    }
    PRINT("CostLess::init: End\n");
}

uint32_t CostLess::less()
{
    test_simplex();
    ++_iter_cnt;
}

bool CostLess::isEnd()
{
    return _iter_cnt >= 1;
}

const char* CostLess::getRes()
{
    PRINT("CostLess::getRes()\n");
    std::stringstream stream("");
    uint32_t cnt = 0;
    std::vector<NodeX>::const_iterator niter;
    for(niter=_x.cbegin();niter!=_x.cend();++niter)
    {
        if(niter->_ix==0)continue;
        std::stack<uint32_t> node_to_visit;
        std::vector<uint32_t> current_path;
        current_path.reserve(_x.size());
        std::vector<uint32_t> min_traffic; //minimum traffic when reach current node by current path
        min_traffic.reserve(_x.size());
        std::vector<bool> visited(_x.size(),false);
        node_to_visit.push(niter->_n);
        while(!node_to_visit.empty())
        {
            uint32_t node_visit = node_to_visit.top();
            PRINT("CostLess::getRes():visiting n(%u)\n",node_visit);
            visited[node_visit] = true;
            node_to_visit.pop();
            const NodeX &node = _x[node_visit];
            if(current_path.empty())
            {
                current_path.push_back(node._n);
                min_traffic.push_back(node._ix);
            }else{
                const NodeX& node_from = _x[current_path.back()];
                for(std::vector<EdgeX>::const_iterator eiter=node_from._out_edge.cbegin();eiter!=node_from._out_edge.cend();++eiter)
                {
                    const EdgeX & edge = *eiter;
                    if( edge._j == node._n  )
                    {
                        if( edge._ix < min_traffic.back() )
                        {
                            min_traffic.push_back( edge._ix );
                        }else{
                            min_traffic.push_back(min_traffic.back());
                        }
                    }
                }
                current_path.push_back(node._n);
            }
            //arrived at a consumer
            if( -1 != node._ci && min_traffic.back() > 0 )
            {
                for(std::vector<uint32_t>::const_iterator piter=current_path.cbegin();piter!=current_path.cend();++piter)
                {
                    stream << *piter <<" ";
                }
                stream << node._ci <<" ";
                if( node._cx < min_traffic.back() )
                {
                    stream << node._cx <<"\n";
                }else{
                    stream << min_traffic.back() <<"\n";
                }
                if( current_path.size() > 1 )
                {
                    current_path.pop_back();
                    min_traffic.pop_back();
                }
                ++cnt;
            }
            if( min_traffic.back() > 0 )
            {
                for(std::vector<EdgeX>::const_iterator eiter=node._out_edge.cbegin();eiter!=node._out_edge.cend();++eiter)
                {
                    const EdgeX& edge = *eiter;
                    if( 0 != edge._ix && !visited[edge._j] )
                    {
                        node_to_visit.push(edge._j);
                    }
                }
            }
        }
    }
    _res_txt<<cnt<<"\n\n";
    _res_txt<<stream.str();
    _res_str = _res_txt.str();
    PRINT("CostLess::getRes():\n%s\n",_res_str.c_str());
    return _res_str.c_str();
}

//You need to complete the function 
void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * filename)
{
	Timer::tic();
    CostLess cost(topo,line_num);
    while( !cost.isEnd() && !Timer::timeout() )
    {
        uint32_t c = cost.less();
        PRINT("Cost:%u\n",c);
    }
    const char* res = cost.getRes();
    PRINT("deploy_server:\n%s\n",res);
    write_result(res,filename);
}
