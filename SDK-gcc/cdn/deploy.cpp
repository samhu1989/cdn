#include "deploy.h"
#include <stack>
#include <cassert>
#include <cmath>
std::time_t Timer::_time_start;
CostLess::CostLess(char * topo[MAX_EDGE_NUM], int line_num)
{
    init(topo,line_num);
    _lambda = 1.0;
    _inf = 1e4;
    _objf_last = getObjFromX();
    _obji_last = 0;
    isBetterIX();
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
    price >> _sp;
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
    updateX();
    if( isConverge() ) //is converged respecting to the continue problem
    {
        _lambda *= 0.5;
        _objf_last = getObjFromX();
        return _obji_last;
    }
    if( isBetterIX() )
    {
        updateIX();
    }
    return _obji_last;
}

bool CostLess::isConverge()
{
    if( getObjFromX() > _objf_last )
    {
        return true;
    }else return false;
}

bool CostLess::isBetterIX()
{
    uint32_t obj = 0;
    //server traffic gradient
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        int32_t nix = std::round(node._cx);
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {

            EdgeX& edge = *eiter;
            assert( node._n == edge._i );
            int32_t eix = std::round(edge._x);
            if( eix < 0 )
            {
                return false;
            }else if ( eix > edge._x_max )
            {
                return false;
            }else
            {
                nix += eix;
            }
            NodeX & cnode = _x[edge._j];
            for(EdgeX::Iter ceiter = cnode._out_edge.begin(); ceiter != cnode._out_edge.end() ; ++ceiter )
            {
                EdgeX& cedge = *ceiter;
                assert( cnode._n == cedge._i );
                if( cedge._j == node._n )
                {
                    nix -= std::round(cedge._x);
                    break;
                }
            }
        }
        if( nix < 0  )
        {
            return false;
        }else if( nix >= 1 )
        {
            obj += _sp;
        }
    }
    if( _obji_last > 0 && obj > _obji_last )
    {
        return false;
    }else _obji_last = obj;
    return true;
}

bool CostLess::isEnd()
{
     if( _lambda < ( 1.0 / ( 1.0 + _inf ) )  )return true;
}

float CostLess::getObjFromX(void)
{
    float obj = 0.0;
    //server traffic gradient
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        if( node._x < 0.0 )
        {
            obj += node._x*(-_inf);
        }else if( node._x <= 1.0 )
        {
            obj += node._x*_sp;
        }else
        {
            obj += node._x*_sp*_lambda;
        }
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            if( edge._x < 0.0 )
            {
                obj += node._x*(-_inf);
            }else if( edge._x > edge._x_max )
            {
                obj += node._x*_inf;
            }else{
                obj += node._x*edge._a;
            }
        }
    }
    return obj;
}

void CostLess::computeD()
{
    //server traffic gradient
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        if( node._x < 0.0 )
        {
            node._dx = -_inf;
        }else if( node._x <= 1.0 )
        {
            node._dx = _sp;
        }else
        {
            node._dx = _sp*_lambda;
        }
    }
    //net traffic gradient
    float max_abs_dx = 1.0;
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
    {
        EdgeX& edge = *eiter;
        if( edge._x < 0.0 )
        {
            edge._dx = -_inf;
        }else if( edge._x > edge._x_max )
        {
            edge._dx = _inf;
        }else{
            edge._dx  = edge._a;
            edge._dx += niter->_dx;
            edge._dx -= _x[edge._j]._dx;
        }
        //for calculate step let the largest step be 1
        if( std::abs(edge._dx) > max_abs_dx )
        {
            max_abs_dx = std::abs(edge._dx);
        }
    }
    //set step so that the largest update be 1
    _step = 1.0 / max_abs_dx;
}

void CostLess::updateX()
{
    //update edge traffic by gradient
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            edge._x -= _step*edge._dx;
        }
    }
    //update server traffic by net traffic
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        node._x = node._cx;
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            assert( node._n == edge._i );
            node._x += edge._x;
            NodeX & cnode = _x[edge._j];
            for(EdgeX::Iter ceiter = cnode._out_edge.begin(); ceiter != cnode._out_edge.end() ; ++ceiter )
            {
                EdgeX& cedge = *ceiter;
                assert( cnode._n == cedge._i );
                if( cedge._j == node._n )
                {
                    node._x -= cedge._x;
                    break;
                }
            }
        }
    }
}

void CostLess::updateIX()
{
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        node._ix = std::round(node._cx);
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            edge._ix = std::round(edge._x);
            assert( node._n == edge._i );
            node._ix += edge._ix;
            NodeX & cnode = _x[edge._j];
            for(EdgeX::Iter ceiter = cnode._out_edge.begin(); ceiter != cnode._out_edge.end() ; ++ceiter )
            {
                EdgeX& cedge = *ceiter;
                assert( cnode._n == cedge._i );
                if( cedge._j == node._n )
                {
                    node._ix -= std::round(cedge._x);
                    break;
                }
            }
        }
    }
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
