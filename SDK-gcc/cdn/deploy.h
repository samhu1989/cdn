#ifndef __ROUTE_H__
#define __ROUTE_H__
#include "lib_io.h"
#include <iostream>
#include <ctime>
#include <cmath>
#include <sstream>
#include <vector>
#include <memory>
void deploy_server(char * graph[MAX_EDGE_NUM], int edge_num, char * filename);
namespace SIMPLX{
typedef unsigned int uint;
typedef enum {
    less_than = '<',
    greater_than = '>',
    equal = '='
} Restriction;
typedef enum {
    single_solution,
    mutiple_solution,
    unbounded,
    unfeasible,
    feasible
}LPP_type;
typedef struct {
    double* data;
    uint cols;
    uint lines;
} Matrix;
typedef struct {
    double* data;
    uint size;
} Vector;
typedef struct {
    Restriction* restrictions_type;
    int* identity_cols_indexes;
    Matrix A;
    Vector c;
    Vector b;
    uint slack_variables;
}PPL;
typedef struct {
    uint* variables_in_base;
    Matrix table;
    Vector costs;
    uint n_variables_in_base;
} Simplex_table;
typedef struct {
    uint* values;
    uint* tmp_base;
    uint variables_per_base;
    uint n_bases;
    uint capacity_in_bases;
} Remembered_bases;
void set_matrix_value(Matrix*, uint, uint, double);
void set_vector_value(Vector*, uint, double);
double get_matrix_value(Matrix*, uint, uint);
double get_vector_value(Vector*, uint);
void copy_matrix_with_line_offset(Matrix*, Matrix*, uint, uint);
void copy_vector(Vector*, Vector*);
double inner_product(Vector*, Vector*);
void sum_vector_with_multiplier(Vector*, Vector*, double);
void divide_vector_by_scalar(Vector*, double);
void get_vector_from_matrix_line(Matrix*, Vector*, uint);
void get_vector_from_matrix_col_with_offset(Matrix*, Vector*, uint, uint);
void print_vector_as_coefficients(Vector*);
void print_PPL(PPL*);
int get_PPL_from_file(FILE*, PPL*);
void expand_PPL(PPL*);
void get_c_b(Simplex_table*, Vector*);
void fill_simplex_table_z_line(Simplex_table*, Vector*);
int get_first_phase_table(Simplex_table*, PPL*);
LPP_type run_simplex(Simplex_table*, int, int);
LPP_type lpp_type_from_solved_table(Simplex_table*);
LPP_type get_second_phase_table(Simplex_table*, PPL*);
void print_other_solutions_from_base_solution(Simplex_table*, Remembered_bases*);
int is_variable_in_base(Simplex_table*, uint);
void put_in_base(Simplex_table*, uint, uint);
void print_simplex_table(Simplex_table*);
int is_same_base(uint*, uint*, uint);
void alloc_remembered_bases(Remembered_bases*, uint);
void realloc_remembered_bases(Remembered_bases*);
void sort_tmp_base(Remembered_bases*);
void remember_tmp_base(Remembered_bases*);
void copy_to_tmp_base(Remembered_bases*, uint*);
int is_base_remembered(Remembered_bases*, uint*, uint, uint);
int test_simplex(void);
}

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
        EdgeX(uint32_t i,uint32_t j,float xm,float cost):
            _i(i),_j(j),_x_max(xm),_a(cost)
        {
            _ix = 0;
        }
        uint32_t _i; //outgoing node
        uint32_t _j; //incoming node
        uint32_t _ix;//integer net traffic
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
                int32_t c,
                uint32_t cx
        ):_n(n),_ci(c),_cx(cx)
        {
            _ix = 0;
            _out_edge.reserve(20);//reserve accoding to maximum number of the edge
        }
        uint32_t _n;
        uint32_t _ix;//integer server traffic
        int32_t _ci;//consumer id
        uint32_t _cx;//consumer traffic
        std::vector<EdgeX> _out_edge; //outgoing edges
    };
    void init(char* topo[MAX_EDGE_NUM],int line_num);
protected:
    void toPPL(void);
private:
    //result text
    std::string _res_str;
    std::stringstream  _res_txt;
    //server cost
    float _sp;
    //node vector
    std::vector<NodeX> _x;
    uint32_t _node_n;
    uint32_t _edge_n;
    uint32_t _c_max;
    uint32_t _c_sum;
    uint32_t _iter_cnt;
    SIMPLX::PPL* _ppl;
    const static double _relax_eps;
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
    static const std::time_t _time_max = 10;
};


#endif
