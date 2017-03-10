#include "deploy.h"
#include <stack>
#include <cassert>
#include <cmath>
#include <limits>
std::time_t Timer::_time_start;
CostLess::CostLess(char * topo[MAX_EDGE_NUM], int line_num):_sum_cx(0.0)
{
    init(topo,line_num);
    _alpha = 1.0 ;
    _lambda = 1.0 / ( _sum_cx * _sum_cx ) ;
    _inf = 1e4;
    _objf_last = getObjFromX();
    printf("CostLess::CostLess:Init Obj=%f\n",_objf_last);
    _obji_last = 0;
    if(!isBetterIX())
    {
        printf("CostLess::CostLess:Bad Init Integer\n");
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
        _x[s]._out_edge.back()._dual_edge = & _x[e]._out_edge.back();
        _x[e]._out_edge.back()._dual_edge = & _x[s]._out_edge.back();
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
        if( _lambda < 0.5 )
        {
            _lambda = 1.0;
        }else if(_lambda = 1.0 )
        {
            _lambda = 1.1;
        }
        _objf_last = getObjFromX();
        printf("CostLess::less:larger lambda:%f\n",_lambda);
    }
    printf("CostLess::less:Float Obj=%f\n",_objf_last);
    _objf_last = getObjFromX();
    if( isBetterIX() )
    {
        updateIX();
//        resetXtoIX();
    }
    printf("CostLess::less:Integer Obj=%u\n",_obji_last);
    return _obji_last;
}

bool CostLess::isConverge()
{
    float obj = getObjFromX();
    if(  obj >= _objf_last )
    {
        printf("Converged %f >= %f \n",obj,_objf_last);
        return true;
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
                obj += eix*edge._a;
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
    bool flag = _lambda > 1.0;
    if( flag )
    {
        printf("CostLess::isEnd()=true,_gamma=%f\n",_lambda);
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
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        float tmp_obj;
        if( node._x <= 0.0 )
        {
            tmp_obj = ( node._x*node._x ) * _sp;
        }
        else
        {
            float f1 = ( node._x*node._x*_alpha );
            float f2 = ( 2.0*std::atan( node._x ) / M_PI );
            tmp_obj = _sp*( ( 1.0 - _lambda )*f1 + _lambda*f2 );
        }
        printf("CostLess::getObjFromX():n(%u)=%f,obj=%f\n",node._n,node._x,tmp_obj);
        node_obj += tmp_obj;
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            if( edge._x < 0.0 )
            {
                net_obj += edge._x*edge._x;
            }else if( edge._x > edge._x_max )
            {
                net_obj += edge._x*edge._x;
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
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        if( node._x <= 0.0 )
        {
            node._dx = _sp*node._x*2.0;
        }
        else
        {
            float d1 = _alpha*2.0*node._x;
            float d2 = 2.0 / M_PI / ( 1.0 + node._x*node._x );
            node._dx = _sp*( ( 1.0 - _lambda )*d1 + _lambda*d2 );
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
        if( edge._x <= 0.0 )
        {
            edge._dx = 2.0*edge._x;
        }else if( edge._x > edge._x_max )
        {
            edge._dx = 2.0*edge._x;
        }else{
            edge._dx = edge._a;
        }
        edge._dx += niter->_dx;
        edge._dx -= _x[edge._j]._dx;
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
    _step = 1.0 / max_abs_dx;
    printf("CostLess::computeD():step:%f,update range(%f,%f)\n",_step,_step*min_abs_dx,0.1);
}

void CostLess::updateX()
{
    printf("CostLess::updateX()\n");
    //update edge traffic by gradient
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            edge._x -= _step*edge._dx;
        }
    }
    printf("CostLess::updateX():set flow to one direction\n");
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            EdgeX& cedge = *edge._dual_edge;
            if( edge._x <= cedge._x )
            {
                cedge._x -= edge._x;
                edge._x = 0.0;
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
            printf("e(%u->%u):x=%f\n",edge._i,edge._j,edge._x);
            assert( node._n == edge._i );
            node._x += edge._x;
            EdgeX& cedge = *edge._dual_edge;
            node._x -= cedge._x;
        }
        printf("n(%u):x=%f\n",node._n,node._x);
    }
}

void CostLess::updateIX()
{
    printf("CostLess::updateIX()\n");
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
            EdgeX& cedge = *edge._dual_edge;
            node._ix -= std::round(cedge._x);
            printf("CostLess::updateIX():e(%u->%u)=%u\n",edge._i,edge._j,edge._ix);
        }
        printf("CostLess::updateIX():n(%u)=%u\n",node._n,node._ix);
    }
}

void CostLess::resetXtoIX()
{
    printf("CostLess::resetXtoIX()\n");
    for(NodeX::Iter niter=_x.begin();niter!=_x.end();++niter)
    {
        NodeX& node = *niter;
        node._x = float(node._ix);
        for(EdgeX::Iter eiter=niter->_out_edge.begin();eiter!=niter->_out_edge.end();++eiter)
        {
            EdgeX& edge = *eiter;
            edge._x = float(edge._ix);
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
            printf("CostLess::getRes():visiting n(%u)\n",node_visit);
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
