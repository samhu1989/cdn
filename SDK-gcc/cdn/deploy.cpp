#include "deploy.h"
#include <stack>
#include <cassert>
#include <cmath>
#include <limits>
std::time_t Timer::_time_start;
CostLess::CostLess(char * topo[MAX_EDGE_NUM], int line_num):_sum_cx(0.0)
{
    init(topo,line_num);
    _lambda = 1.0 / ( _sum_cx * _sum_cx ) ;
    _inf = 1e4;
    _objf_last = getObjFromX();
    printf("CostLess::CostLess:Init Obj=%f\n",_objf_last);
    allow_go_up_ = _go_up_max;
    _obji_last = 0;
    if(!isBetterIX())
    {
        printf("Bad Init Integer");
    }
    printf("CostLess::CostLess:Init Integer Obj=%u\n",_obji_last);
}

CostLess::~CostLess()
{

}

void CostLess::init(char* topo[MAX_EDGE_NUM],int line_num)
{
    printf("CostLess::init\n");
    std::stringstream head(topo[0]);
    uint32_t node_n,edge_n,c_n;
    printf("CostLess::init: get node_n edge_n c_n\n");
    head >> node_n >> edge_n >> c_n;
    printf("CostLess::init: node_n=%u edge_n=%u c_n=%u\n",node_n,edge_n,c_n);
    //reserve for nodes
    _x.reserve(node_n);
    //create nodes
    for(uint32_t n_i = 0 ; n_i < node_n ; ++n_i )
    {
        _x.emplace_back(n_i,0.0f,0.0f,-1,0.0f);
    }
    printf("CostLess::init: get server price\n");
    std::stringstream price(topo[2]);
    uint32_t sp;
    price >> sp;
    _sp = sp;
    printf("CostLess::init: _sp=%u\n",uint32_t(_sp));
    printf("CostLess::init: get link information\n");
    for( uint32_t link_i = 0; link_i < edge_n ; ++link_i )
    {
        std::stringstream link_txt( topo[4+link_i] );
        uint32_t s,e,max_band,cost;
        link_txt >> s >> e >> max_band >> cost;
        _x[s]._out_edge.emplace_back(s,e,0.0f,0.0f,float(max_band),float(cost));
        _x[e]._out_edge.emplace_back(e,s,0.0f,0.0f,float(max_band),float(cost));
    }
    printf("CostLess::init: get client information\n");
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
        _sum_cx += c_traffic;
    }
    printf("CostLess::init: End\n");
}

uint32_t CostLess::less()
{
    computeD();
    updateX();
    if( isConverge() ) //is converged respecting to the continue problem
    {
        _lambda *= 1.1;
        printf("CostLess::less:lower lambda:%f\n",_lambda);
        return _obji_last;
    }
    printf("CostLess::less:Float Obj=%f\n",_objf_last);
    _objf_last = getObjFromX();
    if( isBetterIX() )
    {
        updateIX();
    }
    printf("CostLess::less:Integer Obj=%u\n",_obji_last);
    return _obji_last;
}

bool CostLess::isConverge()
{
    float obj = getObjFromX();
    if(  obj > _objf_last )
    {
        if( !allow_go_up_ )
        {
            allow_go_up_ = _go_up_max;
            printf("Converged %f > %f \n",obj,_objf_last);
            return true;
        }else{
            --allow_go_up_;
        }
    }
    return false;
}

bool CostLess::isBetterIX()
{
    printf("CostLess::isBetterIX()\n");
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
                printf("CostLess::isBetterIX():edge traffic lower zero\n");
                return false;
            }else if ( eix > edge._x_max )
            {
                printf("CostLess::isBetterIX():edge traffic above max\n");
                printf("CostLess::isBetterIX():edge(%u->%u)=%u>%u\n",edge._i,edge._j,eix,uint32_t(edge._x_max));
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
            printf("CostLess::isBetterIX():node traffic lower zero\n");
            return false;
        }else if( nix >= 1 )
        {
            obj += _sp;
        }
    }
    if( _obji_last == 0 )
    {
        _obji_last = obj;
        return true;
    }else if(  obj >= _obji_last )
    {
        printf("CostLess::isBetterIX():same integer result\n");
        return false;
    }else{
        _obji_last = obj;
        return true;
    }
}

bool CostLess::isEnd()
{
    bool flag = _lambda >= 1.0;
    if( flag )
    {
        printf("CostLess::isEnd()=true,_lambda=%f\n",_lambda);
        return true;
    }
    else return false;
}

float CostLess::getObjFromX(void)
{
    printf("CostLess::getObjFromX()\n");
    float node_obj = 0.0;
    float net_obj = 0.0;
    //server traffic gradient
    float dx = std::sqrt( 1.0 / _lambda );
    float dy = 1.0 - _lambda;
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        float tmp_obj;
        if( node._x <= 0.0 )
        {
            tmp_obj = ( node._x*node._x ) * _sp;
        }
        else if( node._x <= dx )
        {
            tmp_obj = ( node._x*node._x*_lambda ) * _sp;
        }else
        {
            tmp_obj = ( dy / ( _sum_cx - 1.0 ) * ( node._x - _sum_cx ) + 1.0 )*_sp;
        }
        printf("CostLess::getObjFromX():n(%u)=%f,dx=%f,dy=%f,obj=%f\n",node._n,node._x,dx,dy,tmp_obj);
        node_obj += tmp_obj;
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            if( edge._x < 0.0 )
            {
                net_obj += edge._x*edge._x;
            }else if( edge._x > edge._x_max )
            {
                net_obj += ( edge._x - edge._x_max )*_inf + edge._a*edge._x_max;
            }else{
                net_obj += edge._x*edge._a;
            }
        }
    }
    printf("CostLess::getObjFromX():node_obj:%f,net_obj:%f\n",node_obj,net_obj);
    return node_obj + net_obj;
}

void CostLess::computeD()
{
    printf("CostLess::computeD()\n");
    //server traffic gradient
    float dx = std::sqrt( 1.0 / _lambda );
    float dy = 1.0 - _lambda;
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        if( node._x <= 0.0 )
        {
            node._dx = _sp*node._x*2.0;
        }
        else if( node._x <= dx )
        {
            node._dx = _sp*_lambda*2.0*node._x;
        }else
        {
            node._dx = _sp * dy / ( _sum_cx - 1.0 );
        }
        printf("n(%u):x=%f,dx=%f\n",node._n,node._x,node._dx);
    }
    //net traffic gradient
    float max_abs_dx = 1.0;
    float min_abs_dx = _inf;
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
    {
        EdgeX& edge = *eiter;
        if( edge._x < 0.0 )
        {
            edge._dx = 2.0*edge._x;
        }else if( edge._x > edge._x_max )
        {
            edge._dx = _inf;
        }else{
            edge._dx  = edge._a;
            edge._dx += niter->_dx;
            edge._dx -= _x[edge._j]._dx;
        }
        printf("e(%u->%u):x=%f,dx=%f\n",edge._i,edge._j,edge._x,edge._dx);
        //for calculate step let the largest step be 1
        if( std::abs(edge._dx) > max_abs_dx )
        {
            max_abs_dx = std::abs(edge._dx);
        }
        if( std::abs(edge._dx) > 0.0 && std::abs(edge._dx) < min_abs_dx )
        {
            min_abs_dx = std::abs(edge._dx);
        }
    }
    //set step so that the largest update be 1
    _step = 0.5 / max_abs_dx;
    printf("CostLess::computeD():step:%f,update range(%f,%f)\n",_step,_step*min_abs_dx,0.1);
}

void CostLess::updateX()
{
    printf("CostLess::updateX()\n");
    float min_net_traffic = _inf;
    //update edge traffic by gradient
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            edge._x -= _step*edge._dx;
            printf("e(%u->%u):x=%f\n",edge._i,edge._j,edge._x);
            if( min_net_traffic > edge._x )
            {
                min_net_traffic = edge._x;
            }
        }
    }
    if( min_net_traffic < 0.0 )
    {
        for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
        {
            for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
            {
                EdgeX& edge = *eiter;
                edge._x -= min_net_traffic;
                printf("e(%u->%u):x=%f\n",edge._i,edge._j,edge._x);
            }
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
        printf("n(%u):x=%f\n",node._n,node._x);
    }
}

void CostLess::updateIX()
{
    printf("CostLess::updateIX()");
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
    printf("CostLess::getRes()\n");
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
                stream << min_traffic.back() <<"\n";
                current_path.pop_back();
                min_traffic.pop_back();
                ++cnt;
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
    _res_txt<<cnt<<"\n\n";
    _res_txt<<stream.str();
    _res_str = _res_txt.str();
    printf("CostLess::getRes():\n%s\n",_res_str.c_str());
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
        printf("Cost:%u\n",c);
    }
    const char* res = cost.getRes();
    printf("deploy_server:\n%s\n",res);
    write_result(res,filename);
}
