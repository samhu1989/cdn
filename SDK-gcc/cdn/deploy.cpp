#include "deploy.h"
#include <stack>
std::time_t Timer::_time_start;
CostLess::CostLess(char * topo[MAX_EDGE_NUM], int line_num)
{
    init(topo,line_num);
}

CostLess::~CostLess()
{
}

void CostLess::init(char* topo[MAX_EDGE_NUM],int line_num)
{
    std::stringstream head(topo[0]);
    uint32_t node_n,edge_n,c_n;
    //net node number link number and consumer number
    head >> node_n >> edge_n >> c_n;
    //reserve for nodes
    _x.reserve(node_n);
    //create nodes
    for(uint32_t n_i = 0 ; n_i < node_n ; ++n_i )
    {
        _x.emplace_back(n_i,0.0f,0.0f,-1,0.0f);
    }
    //server price
    std::stringstream price(topo[2]);
    price >> sp_;
    //link info
    for( uint32_t link_i = 0; link_i < edge_n ; ++link_i )
    {
        std::stringstream link_txt( topo[4+2*link_i] );
        uint32_t s,e,max_band,cost;
        link_txt >> s >> e >> max_band >> cost;
        _x[s]._out_edge.emplace_back(s,e,0.0f,0.0f,float(max_band),float(cost));
        _x[e]._out_edge.emplace_back(e,s,0.0f,0.0f,float(max_band),float(cost));
    }
    //consumer info
    for( uint32_t c_i = 0; c_i < c_n; ++c_i)
    {
        std::stringstream c_txt( topo[4+2*edge_n+2*c_i] );
        uint32_t c_id,net_id,c_traffic;
        c_txt >> c_id >> net_id >> c_traffic;
        NodeX& node = _x[net_id];
        node._ci = c_id;
        node._cx = c_traffic;
        node._x = c_traffic; //init the node result by placing a server at the connected node
        node._ix = c_traffic;
    }
}

uint32_t CostLess::less()
{
    computeD();
    projectD();
    update();
    return uint32_t(computeObj());
}

bool CostLess::isEnd()
{
     return true;
}

float CostLess::computeObj()
{
    ;
}

void CostLess::computeD()
{
    ;
}

void CostLess::projectD()
{
    ;
}

void CostLess::update()
{
    ;
}

const char* CostLess::getRes()
{
    std::stringstream stream;
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
                            min_traffic.push_back(edge._ix);
                        }else{
                            min_traffic.push_back(min_traffic.back());
                        }
                    }
                }
                current_path.push_back(node._n);
            }
            //arrived at a consumer
            if( -1 != node._ci )
            {
                for(std::vector<uint32_t>::const_iterator piter=current_path.cbegin();piter!=current_path.cend();++piter)
                {
                    stream << *piter <<" ";
                }
                stream << node._ci <<" ";
                stream << min_traffic.back() <<"\n\n";
                current_path.pop_back();
                min_traffic.pop_back();
            }else{
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
    _res_txt.flush();
    _res_txt<<cnt<<"\n\n";
    _res_txt<<stream.str()<<std::ends;
    return _res_txt.str().c_str();
}

//You need to complete the function 
void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * filename)
{
	Timer::tic();
    CostLess cost(topo,line_num);
    while( !cost.isEnd() && !Timer::timeout() )
    {
        uint32_t c = cost.less();
        printf("cost:%u",c);
    }
    write_result(cost.getRes(),filename);
}
